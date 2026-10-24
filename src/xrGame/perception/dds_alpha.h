#pragma once

namespace perception
{
	struct uv_rect
	{
		float u0, v0, u1, v1;

		void invalidate() { u0 = v0 = flt_max; u1 = v1 = -flt_max; }
		void modify(float u, float v)
		{
			if (u < u0) u0 = u;
			if (v < v0) v0 = v;
			if (u > u1) u1 = u;
			if (v > v1) v1 = v;
		}
		bool valid() const { return u1 >= u0 && v1 >= v0; }
	};

	struct dds_info
	{
		u32 width, height, mips;
		u32 fourcc; // 0 for uncompressed
		u32 bits;   // bpp for uncompressed
		u32 amask;
	};

	bool dds_probe(const u8* data, u32 size, dds_info& out);

	// Mean alpha over a UV rect, 0..1, estimated on a bounded sample grid. UVs outside [0,1] are
	// wrapped; an invalid rect probes the whole image. Returns false if unparseable.
	//
	// min_res picks the smallest mip at least that wide - cheap, and exact for 8-bit alpha, whose
	// mips box-filter linearly. It is ignored for DXT1, which is always read at mip 0 because its
	// 1-bit alpha forces mip generation to threshold; see the note in dds_mean_alpha.
	//
	// out_gap, if non-null, receives the gap fraction: the share of the rect's 64 sample blocks whose
	// coverage is below GAP_COVERAGE. Two textures with equal mean alpha differ here when one spreads
	// its coverage evenly (fine leaves, gap ~0) and the other concentrates it (bare branches, gap
	// high) - which is what decides whether a ray or a target sample finds a clear path.
	bool dds_mean_alpha(const u8* data, u32 size, const uv_rect& uv, float& out_alpha,
	                    u32 min_res = 64, float* out_gap = 0);
}
