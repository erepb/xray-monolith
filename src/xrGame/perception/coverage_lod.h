#pragma once

namespace perception
{
	// Share of a texture drawn at the alpha cutoff, as a mip pyramid: level 0 is the finest grid and
	// each level above averages 2x2 cells of the one below, down to 1x1, which is the texture's mean
	// coverage. See CANOPY-DISCRETE-DESIGN 12.
	class coverage_lod
	{
	public:
		// base: res x res cells, row-major, 0..255; res a power of two.
		void build(const xr_vector<u8>& base, u32 res);

		bool valid() const { return 0 != m_res; }
		u32 res() const { return m_res; }
		u32 levels() const { return u32(m_offset.size()); }
		u32 bytes() const { return u32(m_data.size()); }

		// The 1x1 level, 0..1.
		float mean() const { return float(m_data[m_offset.back()]) / 255.f; }

		// 0..1 around (u, v), UVs wrapped. level is fractional, 0 = base; outside the pyramid it is
		// clamped to the base or the 1x1 level. Blends the two nearest levels, each read bilinearly.
		float sample(float u, float v, float level) const;

	private:
		float bilinear(u32 level, float u, float v) const;

		xr_vector<u8> m_data;    // every level, finest first
		xr_vector<u32> m_offset; // per level, into m_data
		u32 m_res = 0;
	};
}
