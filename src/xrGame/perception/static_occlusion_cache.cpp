#include "stdafx.h"
#include "static_occlusion_cache.h"

namespace perception
{
	// Per-axis, matching the baked field's own resolution: slots are 2 m in XZ (0.5 m is already four
	// times finer than the data), slab height is stored in 10 cm units.
	static const float CELL_XZ = 0.50f;
	static const float CELL_Y = 0.10f;

	static IC int quant(float v, float cell)
	{
		return iFloor(v / cell);
	}

	static IC u32 mix(u32 h, int v)
	{
		h ^= u32(v) * 0x9E3779B9u;
		return (h << 5) | (h >> 27);
	}

	void static_occlusion_cache::stats::reset()
	{
		probes = depth_hits = evictions = 0;
	}

	static_occlusion_cache::static_occlusion_cache()
	{
		m_stats.reset();
		clear();
	}

	void static_occlusion_cache::clear()
	{
		m_slots.resize(CAPACITY);
		for (u32 i = 0; i < m_slots.size(); ++i)
			m_slots[i].used = false;
	}

	u32 static_occlusion_cache::slot_of(const Fvector& from, const Fvector& to) const
	{
		u32 h = 0x811C9DC5u;
		h = mix(h, quant(from.x, CELL_XZ));
		h = mix(h, quant(from.y, CELL_Y));
		h = mix(h, quant(from.z, CELL_XZ));
		h = mix(h, quant(to.x, CELL_XZ));
		h = mix(h, quant(to.y, CELL_Y));
		h = mix(h, quant(to.z, CELL_XZ));
		return h & (CAPACITY - 1);
	}

	static IC bool same_cell(const Fvector& a, const Fvector& b)
	{
		return quant(a.x, CELL_XZ) == quant(b.x, CELL_XZ)
			&& quant(a.y, CELL_Y) == quant(b.y, CELL_Y)
			&& quant(a.z, CELL_XZ) == quant(b.z, CELL_XZ);
	}

	bool static_occlusion_cache::lookup(const Fvector& from, const Fvector& to, float& depth) const
	{
		++m_stats.probes;

		const u32 base = slot_of(from, to);
		for (u32 p = 0; p < PROBE_LIMIT; ++p)
		{
			const entry& e = m_slots[(base + p) & (CAPACITY - 1)];
			if (!e.used)
				break;
			if (!same_cell(e.from, from) || !same_cell(e.to, to))
				continue;

			depth = e.foliage_tau;
			++m_stats.depth_hits;
			return true;
		}
		return false;
	}

	void static_occlusion_cache::store(const entry& e)
	{
		const u32 base = slot_of(e.from, e.to);

		u32 victim = base;
		u32 oldest = u32(-1);
		for (u32 p = 0; p < PROBE_LIMIT; ++p)
		{
			const u32 i = (base + p) & (CAPACITY - 1);
			entry& s = m_slots[i];
			if (!s.used || (same_cell(s.from, e.from) && same_cell(s.to, e.to)))
			{
				s = e;
				s.used = true;
				return;
			}
			if (s.stamp < oldest)
			{
				oldest = s.stamp;
				victim = i;
			}
		}

		m_slots[victim] = e;
		m_slots[victim].used = true;
		++m_stats.evictions;
	}
}
