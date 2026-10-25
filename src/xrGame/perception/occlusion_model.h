#pragma once

#include "occlusion_types.h"

namespace perception
{
	// Every tunable the occlusion path has, in one place.
	//
	// foliage_k used to carry the whole magnitude, because the bake stored a normalised 0..1 blend
	// weight that no constant could honestly convert into extinction. The field now stores a real
	// extinction coefficient in 1/m, derived from quad area, canopy height, atlas alpha and the
	// instance count the renderer will actually place - so k is a trim around 1.0 for taste, not a
	// fudge factor standing in for the physics.
	struct occlusion_model
	{
		float ray_extend;  // metres past the target; 0.20 reproduces v1 (Feel_Vision.cpp:212)
		float foliage_k;   // trim on the baked grass extinction; 1.0 = take the bake at its word
		float canopy_k;    // same, for tree canopy
		float atmosphere_k; // same, on the measured fog curve: T^k, i.e. exp(-k * tau)
		float max_slab_h;  // metres; clamps the 11-22 m slab outliers Stage 1 measured
		u32 resolve_budget;      // grass slots resolved per frame, engine-wide
		u32 resolve_cache_slots; // resolved grass slots retained
		float alpha_ref;         // 0..1, the detail shader's alpha-test cutoff (def_aref)

		occlusion_model() { set_defaults(); }

		void set_defaults();
		void load();

		float foliage_transmittance(float tau) const;
		float canopy_transmittance(float tau) const;
		float atmosphere_transmittance(float t) const;
	};
}
