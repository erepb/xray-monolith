#include "stdafx.h"
#include "coverage_lod.h"

namespace perception
{
	void coverage_lod::build(const xr_vector<u8>& base, u32 res)
	{
		m_data.clear();
		m_offset.clear();
		m_res = 0;
		if (!res || (res & (res - 1)) || base.size() != size_t(res) * res)
			return;

		m_res = res;
		m_offset.push_back(0);
		m_data = base;
		for (u32 n = res; n > 1; n /= 2)
		{
			const u32 src = m_offset.back();
			const u32 h = n / 2;
			m_offset.push_back(u32(m_data.size()));
			m_data.resize(m_data.size() + size_t(h) * h);
			for (u32 y = 0; y < h; ++y)
				for (u32 x = 0; x < h; ++x)
				{
					const u8* r0 = &m_data[src + (2 * y) * n + 2 * x];
					const u8* r1 = r0 + n;
					m_data[m_offset.back() + y * h + x] = u8((u32(r0[0]) + r0[1] + r1[0] + r1[1] + 2) / 4);
				}
		}
	}

	float coverage_lod::bilinear(u32 level, float u, float v) const
	{
		const u32 n = m_res >> level;
		const u8* g = &m_data[m_offset[level]];

		const float x = (u - floorf(u)) * float(n) - .5f;
		const float y = (v - floorf(v)) * float(n) - .5f;
		const float fx0 = floorf(x), fy0 = floorf(y);
		const float fx = x - fx0, fy = y - fy0;

		const u32 mask = n - 1;
		const u32 x0 = u32(int(fx0)) & mask, y0 = u32(int(fy0)) & mask;
		const u32 x1 = (x0 + 1) & mask, y1 = (y0 + 1) & mask;

		const float a = float(g[y0 * n + x0]) * (1.f - fx) + float(g[y0 * n + x1]) * fx;
		const float b = float(g[y1 * n + x0]) * (1.f - fx) + float(g[y1 * n + x1]) * fx;
		return (a * (1.f - fy) + b * fy) / 255.f;
	}

	float coverage_lod::sample(float u, float v, float level) const
	{
		const float top = float(levels() - 1);
		if (!(level > 0.f))
			return bilinear(0, u, v);
		if (level >= top)
			return mean();

		const u32 l0 = u32(level);
		const float f = level - float(l0);
		const float a = bilinear(l0, u, v);
		return f > 0.f ? a + (bilinear(l0 + 1, u, v) - a) * f : a;
	}
}
