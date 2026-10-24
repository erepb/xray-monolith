#include "stdafx.h"
#include "dds_alpha.h"

namespace perception
{
	static const u32 DDS_MAGIC = 0x20534444; // 'DDS '
	static const u32 HDR_SIZE = 128;

	static const u32 FOURCC_DXT1 = 0x31545844;
	static const u32 FOURCC_DXT3 = 0x33545844;
	static const u32 FOURCC_DXT5 = 0x35545844;

	// grid resolution per axis when probing a rect; 32x32 puts the sampling error near 1.5%, well
	// under the u8 quantisation the density field applies afterwards
	static const u32 SAMPLES_PER_AXIS = 32;

	static const u32 GAP_BLOCKS_PER_AXIS = 8;   // 8x8 contiguous blocks over the rect
	static const u32 GAP_SAMPLES_PER_AXIS = 4;  // 4x4 lattice inside each block -> 1024 total
	static const float GAP_COVERAGE = 0.1f;     // a block this empty is a hole a sample can pass

	static_assert(GAP_BLOCKS_PER_AXIS * GAP_SAMPLES_PER_AXIS == SAMPLES_PER_AXIS,
	              "gap block/lattice split must keep the same per-axis sample budget");

	static IC u32 rd32(const u8* p, u32 off) { return *(const u32*)(p + off); }

	static IC u32 block_bytes(u32 fourcc)
	{
		switch (fourcc)
		{
		case FOURCC_DXT1: return 8;
		case FOURCC_DXT3:
		case FOURCC_DXT5: return 16;
		default: return 0;
		}
	}

	static const u32 MAX_DIM = 16384; // keeps every intermediate below 2^31 for all accepted formats

	static IC u64 level_bytes(const dds_info& i, u32 w, u32 h)
	{
		const u64 w64 = w, h64 = h;
		if (i.fourcc)
		{
			const u64 bx = _max(u64(1), (w64 + 3) / 4);
			const u64 by = _max(u64(1), (h64 + 3) / 4);
			return bx * by * block_bytes(i.fourcc);
		}
		return w64 * h64 * (i.bits / 8);
	}

	bool dds_probe(const u8* data, u32 size, dds_info& out)
	{
		if (!data || size < HDR_SIZE || rd32(data, 0) != DDS_MAGIC)
			return false;

		out.height = rd32(data, 12);
		out.width = rd32(data, 16);
		out.mips = rd32(data, 28);
		const u32 pf_flags = rd32(data, 80);
		out.fourcc = (pf_flags & 0x4) ? rd32(data, 84) : 0;
		out.bits = rd32(data, 88);
		out.amask = (pf_flags & (0x1 | 0x2)) ? rd32(data, 104) : 0;

		if (!out.width || !out.height || out.width > MAX_DIM || out.height > MAX_DIM)
			return false;
		if (out.fourcc && !block_bytes(out.fourcc))
			return false; // DX10 header and friends: not handled
		if (!out.fourcc && out.bits != 32)
			return false;

		if (out.mips == 0)
			out.mips = 1;
		return true;
	}

	// --- per-texel alpha fetch -------------------------------------------------

	static IC u32 dxt1_alpha(const u8* blk, u32 tx, u32 ty)
	{
		const u16 c0 = *(const u16*)(blk + 0);
		const u16 c1 = *(const u16*)(blk + 2);
		if (c0 > c1)
			return 255; // opaque mode: no alpha at all

		const u32 bits = rd32(blk, 4);
		const u32 idx = (bits >> ((ty * 4 + tx) * 2)) & 3;
		return (idx == 3) ? 0 : 255;
	}

	static IC u32 dxt3_alpha(const u8* blk, u32 tx, u32 ty)
	{
		const u32 i = ty * 4 + tx;
		const u32 nib = (blk[i >> 1] >> ((i & 1) ? 4 : 0)) & 0xF;
		return nib * 17; // 0..15 -> 0..255
	}

	static IC u32 dxt5_alpha(const u8* blk, u32 tx, u32 ty)
	{
		const u32 a0 = blk[0];
		const u32 a1 = blk[1];

		u64 bits = 0;
		for (u32 b = 0; b < 6; ++b)
			bits |= u64(blk[2 + b]) << (8 * b);

		const u32 idx = u32((bits >> ((ty * 4 + tx) * 3)) & 7);
		if (idx == 0) return a0;
		if (idx == 1) return a1;

		if (a0 > a1)
			return ((8 - idx) * a0 + (idx - 1) * a1) / 7;

		if (idx == 6) return 0;
		if (idx == 7) return 255;
		return ((6 - idx) * a0 + (idx - 1) * a1) / 5;
	}

	static IC u32 texel_alpha(const dds_info& i, const u8* lvl, u32 w, u32 x, u32 y)
	{
		if (!i.fourcc)
		{
			const u32 px = rd32(lvl, (y * w + x) * 4);
			if (!i.amask)
				return 255;
			// alpha mask is contiguous; shift down by its trailing zero count
			u32 shift = 0, m = i.amask;
			while (!(m & 1)) { m >>= 1; ++shift; }
			const u32 mw = i.amask >> shift; // mask width, >=1 (amask != 0 checked above)
			return ((px & i.amask) >> shift) * 255 / mw;
		}

		const u32 bx = _max(1u, (w + 3) / 4);
		const u32 bsz = block_bytes(i.fourcc);
		const u8* blk = lvl + ((y / 4) * bx + (x / 4)) * bsz;
		const u32 tx = x & 3, ty = y & 3;

		switch (i.fourcc)
		{
		case FOURCC_DXT1: return dxt1_alpha(blk, tx, ty);
		case FOURCC_DXT3: return dxt3_alpha(blk, tx, ty);
		default: return dxt5_alpha(blk, tx, ty);
		}
	}

	// --------------------------------------------------------------------------

	bool dds_mean_alpha(const u8* data, u32 size, const uv_rect& uv, float& out_alpha, u32 min_res,
	                    float* out_gap)
	{
		dds_info i;
		if (!dds_probe(data, size, i))
			return false;

		// DXT1 carries one bit of alpha per texel behind a per-block mode flag, so whoever generated
		// the mips had to threshold rather than average - and the vanilla atlases disagree about which
		// way to round. Measured on the 64x64 mip against mip 0: l07_military reads 0.099 vs 0.171 and
		// decays to zero, k00_marsh reads 0.845 vs 0.193 and saturates to one. Only mip 0 means
		// anything for this format. Formats with 8-bit alpha box-filter linearly, so their mips keep
		// the mean and stay cheap to read.
		if (FOURCC_DXT1 == i.fourcc)
			min_res = i.width;

		// walk down the mip chain while the next level is still wide enough
		u32 w = i.width, h = i.height, off = HDR_SIZE;
		for (u32 m = 1; m < i.mips; ++m)
		{
			const u32 nw = _max(1u, w / 2), nh = _max(1u, h / 2);
			if (nw < min_res)
				break;
			const u64 sz = level_bytes(i, w, h);
			if (u64(off) + sz + level_bytes(i, nw, nh) > u64(size))
				break;
			off += u32(sz);
			w = nw;
			h = nh;
		}
		if (u64(off) + level_bytes(i, w, h) > u64(size))
			return false;

		const u8* lvl = data + off;

		u32 x0 = 0, x1 = w, y0 = 0, y1 = h;
		if (uv.valid())
		{
			// wrap into [0,1); each axis is restricted independently so a rect that wraps in one
			// axis doesn't fall back to a whole-image probe on the other
			const float du = uv.u1 - uv.u0, dv = uv.v1 - uv.v0;
			if (du < 1.f)
			{
				const float fu = uv.u0 - floorf(uv.u0);
				x0 = u32(fu * w);
				x1 = x0 + _max(1u, u32(du * w + 0.5f));
			}
			if (dv < 1.f)
			{
				const float fv = uv.v0 - floorf(uv.v0);
				y0 = u32(fv * h);
				y1 = y0 + _max(1u, u32(dv * h + 0.5f));
			}
		}

		// Sampling mip 0 means the rect can be millions of texels, so step across it instead of
		// reading it whole: a regular grid is an unbiased estimator of coverage and bounds a
		// whole-atlas probe and a single-quad probe to the same cost. Small rects are read densely.
		//
		// The grid is grouped into GAP_BLOCKS_PER_AXIS^2 contiguous neighbourhoods (each holding a
		// GAP_SAMPLES_PER_AXIS^2 lattice) rather than spread evenly, because a uniformly-strided grid
		// cannot resolve structure smaller than its stride - gap fraction needs per-block coverage.
		// Total sample count and cost are unchanged from a flat SAMPLES_PER_AXIS^2 grid.
		const u32 rw = x1 - x0, rh = y1 - y0;
		const u32 nbx = _min(GAP_BLOCKS_PER_AXIS, rw);
		const u32 nby = _min(GAP_BLOCKS_PER_AXIS, rh);

		u32 total = 0, count = 0, gap_blocks = 0;
		for (u32 by = 0; by < nby; ++by)
		{
			const u32 by0 = y0 + (by * rh) / nby;
			const u32 by1 = y0 + ((by + 1) * rh) / nby;
			const u32 nsy = _min(GAP_SAMPLES_PER_AXIS, by1 - by0);
			const u32 syb = _max(1u, (by1 - by0) / nsy);

			for (u32 bx = 0; bx < nbx; ++bx)
			{
				const u32 bx0 = x0 + (bx * rw) / nbx;
				const u32 bx1 = x0 + ((bx + 1) * rw) / nbx;
				const u32 nsx = _min(GAP_SAMPLES_PER_AXIS, bx1 - bx0);
				const u32 sxb = _max(1u, (bx1 - bx0) / nsx);

				u32 b_total = 0, b_count = 0;
				for (u32 y = by0; y < by1; y += syb)
				{
					const u32 yy = y % h;
					for (u32 x = bx0; x < bx1; x += sxb)
					{
						b_total += texel_alpha(i, lvl, w, x % w, yy);
						++b_count;
					}
				}

				total += b_total;
				count += b_count;
				if (b_count && float(b_total) < GAP_COVERAGE * 255.f * float(b_count))
					++gap_blocks;
			}
		}

		if (!count)
			return false;

		out_alpha = float(total) / float(count) / 255.f;
		if (out_gap)
			*out_gap = float(gap_blocks) / float(nbx * nby);
		return true;
	}
}
