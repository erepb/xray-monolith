#pragma once

#include "occlusion_types.h"

namespace perception
{
	// Every tunable the occlusion path has, in one place. The channels measure optical depth from
	// real placement, geometry and texture coverage, so each *_k is a trim around 1.0, not a fitted
	// constant.
	struct occlusion_model
	{
		float ray_extend;  // metres past the target; 0.20 reproduces v1 (Feel_Vision.cpp:212)
		float foliage_k;   // trim on the baked grass extinction; 1.0 = take the bake at its word
		float canopy_k;    // same, for tree canopy
		float canopy_cone_eye;    // metres; sight-line width at the observer's eye
		float canopy_cone_target; // metres; sight-line width at the target point
		float atmosphere_k; // same, on the measured fog curve: T^k, i.e. exp(-k * tau)
		float max_slab_h;  // metres; clamps the terrain relief of a slot, which reaches 11-22 m on cliffs
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
