#pragma once

#include "occlusion_types.h"

namespace perception
{
	// Memo for the level-static half of a trace, keyed on a quantized endpoint pair.
	//
	// Two tolerances, not one. Foliage optical depth is an integral over a smooth field and is the
	// expensive half (the march resolves and intersects every slot the ray crosses), so it is reused across the whole
	// quantization cell. Static geometry occlusion is a binary edge test - half a metre laterally is
	// the difference between seeing past a trunk and not - so it is reused only when the endpoints
	// agree almost exactly.
	//
	// The cell is anisotropic because the baked field is: slots are 2 m in XZ but slabs are 0.6 m
	// median tall, and cell::slab_h is stored in 10 cm units. Quantizing Y at 0.5 m would be 83% of a
	// median slab, enough to move a ray from inside the grass to above it.
	class static_occlusion_cache
	{
	public:
		static_occlusion_cache();

		struct entry
		{
			Fvector from, to; // exact endpoints this entry was computed from
			float foliage_tau;

			// False when the march hit the resolve budget and over-estimated on purpose. store()
			// owns `used` and forces it true, so gating on that flag silently did nothing - and the
			// geom half of the same entry is perfectly valid regardless, so it wants its own flag.
			bool tau_valid;
			float geom[occ_class_count]; // static geometry contribution; creature, foliage, canopy and atmosphere stay 1
			u32 stamp;
			bool used;
		};

		struct stats
		{
			u32 probes;
			u32 depth_hits;
			u32 geom_hits;
			u32 evictions;
			void reset();
		};

		// Fills `depth` on a cell hit. Fills `geom` additionally when the stored endpoints are within
		// GEOM_EPS. Returns which halves were served.
		void lookup(const Fvector& from, const Fvector& to,
		            float& depth, bool& got_depth,
		            float* geom, bool& got_geom) const;

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
