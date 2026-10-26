#pragma once

namespace perception
{
	// Memo for the grass optical depth of a trace, keyed on a quantized endpoint pair. Depth is an
	// integral over a smooth field and the expensive part (the march resolves and intersects every slot
	// the ray crosses), so it is reused across the whole quantization cell.
	class static_occlusion_cache
	{
	public:
		static_occlusion_cache();

		struct entry
		{
			Fvector from, to; // exact endpoints this entry was computed from
			float foliage_tau;
			u32 stamp;
			bool used;
		};

		struct stats
		{
			u32 probes;
			u32 depth_hits;
			u32 evictions;
			void reset();
		};

		// Fills `depth` and returns true on a cell hit.
		bool lookup(const Fvector& from, const Fvector& to, float& depth) const;

		void store(const entry& e);
		void clear();

		const stats& get_stats() const { return m_stats; }
		void reset_stats() { m_stats.reset(); }

	private:
		u32 slot_of(const Fvector& from, const Fvector& to) const;

		enum { CAPACITY = 4096, PROBE_LIMIT = 4 };

		xr_vector<entry> m_slots;
		mutable stats m_stats;
	};
}
