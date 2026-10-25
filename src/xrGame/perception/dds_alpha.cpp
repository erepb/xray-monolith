#include "stdafx.h"
#include "dds_alpha.h"

namespace perception
{
	static const u32 DDS_MAGIC = 0x20534444; // 'DDS '
	static const u32 HDR_SIZE = 128;

	static const u32 FOURCC_DXT1 = 0x31545844;
	static const u32 FOURCC_DXT3 = 0x33545844;
	static const u32 FOURCC_DXT5 = 0x35545844;

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

	bool dds_alpha_view::open(const u8* data, u32 size)
	{
		m_lvl = 0;
		if (!dds_probe(data, size, m_info))
			return false;
		if (u64(HDR_SIZE) + level_bytes(m_info, m_info.width, m_info.height) > u64(size))
			return false;
		m_lvl = data + HDR_SIZE;
		return true;
	}

	u32 dds_alpha_view::alpha(float u, float v) const
	{
		VERIFY(m_lvl);
		const u32 w = m_info.width, h = m_info.height;
		const u32 x = _min(u32((u - floorf(u)) * float(w)), w - 1);
		const u32 y = _min(u32((v - floorf(v)) * float(h)), h - 1);
		return texel_alpha(m_info, m_lvl, w, x, y);
	}

	// --------------------------------------------------------------------------

	static const u32 CELL_BLOCKS_MAX = 8;  // blocks per cell axis read in full; past this, a subset
	static const u32 CELL_BLOCKS_READ = 4; // blocks per cell axis in that subset
	static const u32 CELL_TEXELS_MAX = 8;  // texel lattice per cell axis where blocks cannot be counted

	static IC u32 popcount32(u32 x)
	{
		x = x - ((x >> 1) & 0x55555555u);
		x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
		return (((x + (x >> 4)) & 0x0f0f0f0fu) * 0x01010101u) >> 24;
	}

	// Texels of a DXT1 block that are drawn: all 16 in opaque mode, otherwise those whose index is not 3.
	static IC u32 dxt1_drawn(const u8* blk)
	{
		const u16 c0 = *(const u16*)(blk + 0);
		const u16 c1 = *(const u16*)(blk + 2);
		if (c0 > c1)
			return 16;
		const u32 bits = rd32(blk, 4);
		return 16 - popcount32(bits & (bits >> 1) & 0x55555555u);
	}

	bool dds_coverage_grid(const u8* data, u32 size, u32 res, float alpha_ref, xr_vector<u8>& out)
	{
		dds_info i;
		if (!dds_probe(data, size, i))
			return false;
		if (!res || (res & (res - 1)) || res > i.width || res > i.height)
			return false;
		if (u64(HDR_SIZE) + level_bytes(i, i.width, i.height) > u64(size))
			return false;

		const u8* lvl = data + HDR_SIZE;
		const u32 w = i.width, h = i.height;
		out.assign(size_t(res) * res, 0);

		if (FOURCC_DXT1 == i.fourcc && 0 == w % (4 * res) && 0 == h % (4 * res))
		{
			const u32 row = w / 4;
			const u32 nbx = w / res / 4, nby = h / res / 4;
			const u32 sx = nbx > CELL_BLOCKS_MAX ? nbx / CELL_BLOCKS_READ : 1;
			const u32 sy = nby > CELL_BLOCKS_MAX ? nby / CELL_BLOCKS_READ : 1;
			for (u32 cy = 0; cy < res; ++cy)
				for (u32 cx = 0; cx < res; ++cx)
				{
					u32 drawn = 0, n = 0;
					for (u32 by = 0; by < nby; by += sy)
					{
						const u8* blk = lvl + (size_t(cy * nby + by) * row + cx * nbx) * 8;
						for (u32 bx = 0; bx < nbx; bx += sx)
						{
							drawn += dxt1_drawn(blk + bx * 8);
							n += 16;
						}
					}
					out[cy * res + cx] = u8((drawn * 255 + n / 2) / n);
				}
			return true;
		}

		const u32 ref = u32(alpha_ref * 255.f + .5f);
		for (u32 cy = 0; cy < res; ++cy)
		{
			const u32 y0 = cy * h / res, y1 = (cy + 1) * h / res;
			const u32 ny = _min(CELL_TEXELS_MAX, y1 - y0);
			for (u32 cx = 0; cx < res; ++cx)
			{
				const u32 x0 = cx * w / res, x1 = (cx + 1) * w / res;
				const u32 nx = _min(CELL_TEXELS_MAX, x1 - x0);
				u32 drawn = 0;
				for (u32 sy = 0; sy < ny; ++sy)
				{
					const u32 y = y0 + ((2 * sy + 1) * (y1 - y0)) / (2 * ny);
					for (u32 sx = 0; sx < nx; ++sx)
					{
						const u32 x = x0 + ((2 * sx + 1) * (x1 - x0)) / (2 * nx);
						if (texel_alpha(i, lvl, w, x, y) >= ref)
							++drawn;
					}
				}
				const u32 n = nx * ny;
				out[cy * res + cx] = u8((drawn * 255 + n / 2) / n);
			}
		}
		return true;
	}
}
