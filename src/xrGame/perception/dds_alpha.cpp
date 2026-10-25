#include "stdafx.h"
#include "dds_alpha.h"

namespace perception
{
	static const u32 DDS_MAGIC = 0x20534444; // 'DDS '
	static const u32 HDR_SIZE = 128;
	static const u32 DX10_EXTRA = 20;

	static const u32 FOURCC_DXT1 = 0x31545844;
	static const u32 FOURCC_DXT3 = 0x33545844;
	static const u32 FOURCC_DXT5 = 0x35545844;
	static const u32 FOURCC_DX10 = 0x30315844;
	static const u32 FOURCC_BC7 = 0x20374342;

	static IC u32 rd32(const u8* p, u32 off) { return *(const u32*)(p + off); }

	static IC u32 block_bytes(u32 fourcc)
	{
		switch (fourcc)
		{
		case FOURCC_DXT1: return 8;
		case FOURCC_DXT3:
		case FOURCC_DXT5:
		case FOURCC_BC7: return 16;
		default: return 0;
		}
	}

	static const u32 MAX_DIM = 16384; // keeps every intermediate below 2^31 for all accepted formats

	static IC u64 level_bytes(const dds_info& i)
	{
		const u64 w64 = i.width, h64 = i.height;
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
		out.data_offset = HDR_SIZE;

		if (out.fourcc == FOURCC_DX10)
		{
			if (size < HDR_SIZE + DX10_EXTRA)
				return false;
			const u32 dxgi = rd32(data, HDR_SIZE);
			const u32 dimension = rd32(data, HDR_SIZE + 4);
			const u32 misc = rd32(data, HDR_SIZE + 8);
			const u32 array_size = rd32(data, HDR_SIZE + 12);
			if (dimension != 3 || array_size > 1 || (misc & 0x4))
				return false;
			out.data_offset = HDR_SIZE + DX10_EXTRA;
			switch (dxgi)
			{
			case 70: case 71: case 72: out.fourcc = FOURCC_DXT1; break;
			case 73: case 74: case 75: out.fourcc = FOURCC_DXT3; break;
			case 76: case 77: case 78: out.fourcc = FOURCC_DXT5; break;
			case 97: case 98: case 99: out.fourcc = FOURCC_BC7; break;
			case 27: case 28: case 29: case 87: case 90: case 91:
				out.fourcc = 0;
				out.bits = 32;
				out.amask = 0xFF000000;
				break;
			default: return false;
			}
		}

		if (!out.width || !out.height || out.width > MAX_DIM || out.height > MAX_DIM)
			return false;
		if (out.fourcc && !block_bytes(out.fourcc))
			return false;
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

	// Tables and bit layout from the D3D11 BC7 format spec (DirectXTex BC6HBC7.cpp).
	static const u16 BC7_PART2[64] = {
		0xcccc, 0x8888, 0xeeee, 0xecc8, 0xc880, 0xfeec, 0xfec8, 0xec80,
		0xc800, 0xffec, 0xfe80, 0xe800, 0xffe8, 0xff00, 0xfff0, 0xf000,
		0xf710, 0x008e, 0x7100, 0x08ce, 0x008c, 0x7310, 0x3100, 0x8cce,
		0x088c, 0x3110, 0x6666, 0x366c, 0x17e8, 0x0ff0, 0x718e, 0x399c,
		0xaaaa, 0xf0f0, 0x5a5a, 0x33cc, 0x3c3c, 0x55aa, 0x9696, 0xa55a,
		0x73ce, 0x13c8, 0x324c, 0x3bdc, 0x6996, 0xc33c, 0x9966, 0x0660,
		0x0272, 0x04e4, 0x4e40, 0x2720, 0xc936, 0x936c, 0x39c6, 0x639c,
		0x9336, 0x9cc6, 0x817e, 0xe718, 0xccf0, 0x0fcc, 0x7744, 0xee22,
	};

	static const u8 BC7_ANCHOR2[64] = {
		15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
		15, 2, 8, 2, 2, 8, 8, 15, 2, 8, 2, 2, 8, 8, 2, 2,
		15, 15, 6, 8, 2, 8, 15, 15, 2, 8, 2, 2, 2, 15, 15, 6,
		6, 2, 6, 8, 15, 15, 2, 2, 15, 15, 15, 15, 15, 2, 2, 15,
	};

	struct bc7_mode
	{
		u8 part_bits, rot_bits, idxmode_bits, pbits, idx_prec, idx_prec2;
		u8 prec[4], precp[4];
	};

	// Modes 4-7; modes 0-3 carry no alpha.
	static const bc7_mode BC7_MODES[4] = {
		{0, 2, 1, 0, 2, 3, {5, 5, 5, 6}, {5, 5, 5, 6}},
		{0, 2, 0, 0, 2, 2, {7, 7, 7, 8}, {7, 7, 7, 8}},
		{0, 0, 0, 2, 4, 0, {7, 7, 7, 7}, {8, 8, 8, 8}},
		{6, 0, 0, 4, 2, 0, {5, 5, 5, 5}, {6, 6, 6, 6}},
	};

	static const u8 BC7_W2[4] = {0, 21, 43, 64};
	static const u8 BC7_W3[8] = {0, 9, 18, 27, 37, 46, 55, 64};
	static const u8 BC7_W4[16] = {0, 4, 9, 13, 17, 21, 26, 30, 34, 38, 43, 47, 51, 55, 60, 64};

	static IC const u8* bc7_weights(u32 prec)
	{
		return prec == 2 ? BC7_W2 : prec == 3 ? BC7_W3 : BC7_W4;
	}

	// Bits [pos, pos + n) of the 128-bit block, n <= 8.
	static IC u32 bc7_get(u64 lo, u64 hi, u32 pos, u32 n)
	{
		const u64 mask = (u64(1) << n) - 1;
		if (pos >= 64)
			return u32((hi >> (pos - 64)) & mask);
		if (pos + n <= 64)
			return u32((lo >> pos) & mask);
		return u32(((lo >> pos) | (hi << (64 - pos))) & mask);
	}

	static IC u32 bc7_lerp(u32 a, u32 b, u32 w)
	{
		return (a * (64 - w) + b * w + 32) >> 6;
	}

	// Reads only the two endpoint values and the index that texel (tx, ty)'s alpha needs.
	static u32 bc7_alpha(const u8* blk, u32 tx, u32 ty)
	{
		u64 lo, hi;
		memcpy(&lo, blk, 8);
		memcpy(&hi, blk + 8, 8);

		u32 mode = 0;
		while (mode < 8 && !((lo >> mode) & 1))
			++mode;
		if (mode >= 8)
			return 0;
		if (mode < 4)
			return 255;

		const bc7_mode& m = BC7_MODES[mode - 4];
		u32 pos = mode + 1;
		const u32 shape = bc7_get(lo, hi, pos, m.part_bits);
		pos += m.part_bits;
		const u32 rot = bc7_get(lo, hi, pos, m.rot_bits);
		pos += m.rot_bits;
		const u32 idxmode = bc7_get(lo, hi, pos, m.idxmode_bits);
		pos += m.idxmode_bits;
		const u32 nep = mode == 7 ? 4 : 2;

		const u32 ch = rot ? rot - 1 : 3;
		u32 ch_pos = pos;
		u32 total = 0;
		for (u32 k = 0; k < 4; ++k)
		{
			if (k == ch)
				ch_pos = pos + total;
			total += nep * m.prec[k];
		}
		const u32 pbase = pos + total;
		const u32 ibase = pbase + m.pbits;

		const u32 i = ty * 4 + tx;
		const u32 region = mode == 7 ? (BC7_PART2[shape] >> i) & 1 : 0;
		u32 e[2];
		for (u32 k = 0; k < 2; ++k)
		{
			const u32 ep = region * 2 + k;
			u32 v = bc7_get(lo, hi, ch_pos + ep * m.prec[ch], m.prec[ch]);
			if (m.pbits && m.prec[ch] != m.precp[ch])
				v = (v << 1) | bc7_get(lo, hi, pbase + ep * m.pbits / nep, 1);
			v = (v << (8 - m.precp[ch])) & 0xFF;
			e[k] = v | (v >> m.precp[ch]);
		}

		const bool set2 = m.idx_prec2 && (rot ? idxmode == 1 : idxmode == 0);
		const u32 iprec = set2 ? m.idx_prec2 : m.idx_prec;
		const u32 anchor2 = BC7_ANCHOR2[shape];
		const bool anchored = !set2 && mode == 7;
		const u32 off = i * iprec - (i > 0 ? 1 : 0) - (anchored && i > anchor2 ? 1 : 0);
		const u32 start = set2 ? ibase + 16 * m.idx_prec - 1 : ibase;
		const bool fix = i == 0 || (anchored && i == anchor2);
		const u32 w = bc7_get(lo, hi, start + off, fix ? iprec - 1 : iprec);
		return bc7_lerp(e[0], e[1], bc7_weights(iprec)[w]);
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
		case FOURCC_BC7: return bc7_alpha(blk, tx, ty);
		default: return dxt5_alpha(blk, tx, ty);
		}
	}

	static const u8* mip0(const u8* data, u32 size, dds_info& i)
	{
		if (!dds_probe(data, size, i) || u64(i.data_offset) + level_bytes(i) > u64(size))
			return 0;
		return data + i.data_offset;
	}

	bool dds_alpha_view::open(const u8* data, u32 size)
	{
		m_lvl = mip0(data, size, m_info);
		return 0 != m_lvl;
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
		const u8* lvl = mip0(data, size, i);
		if (!lvl || !res || (res & (res - 1)) || res > i.width || res > i.height)
			return false;

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
