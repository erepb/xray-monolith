#include "stdafx.h"
#include "occlusion_model.h"
#include "perception_config.h"

namespace perception
{
	static LPCSTR CFG_SECTION = "occlusion_model";

	void occlusion_model::set_defaults()
	{
		foliage_k = 1.00f;
		canopy_k = 1.00f;
		canopy_cone_eye = 0.05f;
		canopy_cone_target = 0.30f;
		atmosphere_k = 1.00f;
		resolve_budget = 64;
		resolve_cache_slots = 32768;
		alpha_ref = 200.f / 255.f;
	}

	void occlusion_model::load()
	{
		set_defaults();

		CInifile* ltx = open_config("occlusion model");
		report_section(ltx, CFG_SECTION);

		foliage_k = cfg_float(ltx, CFG_SECTION, "foliage_k", foliage_k, 0.f, 100.f);
		canopy_k = cfg_float(ltx, CFG_SECTION, "canopy_k", canopy_k, 0.f, 100.f);
		canopy_cone_eye = cfg_float(ltx, CFG_SECTION, "canopy_cone_eye", canopy_cone_eye, 0.01f, 1.f);
		canopy_cone_target = cfg_float(ltx, CFG_SECTION, "canopy_cone_target", canopy_cone_target, 0.01f, 2.f);
		atmosphere_k = cfg_float(ltx, CFG_SECTION, "atmosphere_k", atmosphere_k, 0.f, 100.f);
		// A zero budget would make every grass slot permanently opaque.
		resolve_budget = cfg_u32(ltx, CFG_SECTION, "resolve_budget", resolve_budget, 1u, 4096u);
		resolve_cache_slots = cfg_u32(ltx, CFG_SECTION, "resolve_cache_slots", resolve_cache_slots, 1024u, 262144u);
		alpha_ref = cfg_float(ltx, CFG_SECTION, "alpha_ref", alpha_ref, 0.f, 1.f);
		xr_delete(ltx);

		Msg("~ [perception] occlusion model: foliage_k=%.2f canopy_k=%.2f atmosphere_k=%.2f alpha_ref=%.3f",
		    foliage_k, canopy_k, atmosphere_k, alpha_ref);
		Msg("~ [perception]   grass resolve budget %u slot(s)/frame, cache %u slot(s)", resolve_budget,
		    resolve_cache_slots);
		Msg("~ [perception]   canopy sight line %.2f m at the eye, %.2f m at the target", canopy_cone_eye,
		    canopy_cone_target);
	}

	float occlusion_model::foliage_transmittance(float tau) const
	{
		if (tau <= 0.f)
			return 1.f;

		// tau already carries the physics - it is the summed -ln of each crossed instance's
		// transmittance, from real placement, real quad area and real atlas alpha. foliage_k is a
		// trim, not a fit: 1.0 means take the measurement at its word.
		return expf(-foliage_k * tau);
	}

	float occlusion_model::canopy_transmittance(float tau) const
	{
		return tau > 0.f ? expf(-canopy_k * tau) : 1.f;
	}

	float occlusion_model::atmosphere_transmittance(float t) const
	{
		if (atmosphere_k <= 0.f || t >= 1.f)
			return 1.f;

		return powf(_max(t, 0.f), atmosphere_k);
	}
}
