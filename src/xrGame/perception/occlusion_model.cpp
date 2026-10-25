#include "stdafx.h"
#include "occlusion_model.h"

namespace perception
{
	static LPCSTR CFG_FILE = "creatures\\perception.ltx";
	static LPCSTR CFG_SECTION = "occlusion_model";

	void occlusion_model::set_defaults()
	{
		ray_extend = 0.20f;
		foliage_k = 1.00f;
		canopy_k = 1.00f;
		atmosphere_k = 1.00f;
		max_slab_h = 4.00f;
		resolve_budget = 64;
		resolve_cache_slots = 8192;
		alpha_ref = 200.f / 255.f;
	}

	void occlusion_model::load()
	{
		set_defaults();

		string_path fn;
		if (!FS.exist(fn, "$game_config$", CFG_FILE))
		{
			Msg("~ [perception] %s absent, occlusion model at defaults", CFG_FILE);
			return;
		}

		CInifile* ltx = xr_new<CInifile>(fn, TRUE);
		if (ltx->section_exist(CFG_SECTION))
		{
			if (ltx->line_exist(CFG_SECTION, "ray_extend")) ray_extend = ltx->r_float(CFG_SECTION, "ray_extend");
			if (ltx->line_exist(CFG_SECTION, "foliage_k")) foliage_k = ltx->r_float(CFG_SECTION, "foliage_k");
			if (ltx->line_exist(CFG_SECTION, "canopy_k")) canopy_k = ltx->r_float(CFG_SECTION, "canopy_k");
			if (ltx->line_exist(CFG_SECTION, "atmosphere_k")) atmosphere_k = ltx->r_float(CFG_SECTION, "atmosphere_k");
			if (ltx->line_exist(CFG_SECTION, "max_slab_h")) max_slab_h = ltx->r_float(CFG_SECTION, "max_slab_h");
			if (ltx->line_exist(CFG_SECTION, "resolve_budget")) resolve_budget = ltx->r_u32(CFG_SECTION, "resolve_budget");
			if (ltx->line_exist(CFG_SECTION, "resolve_cache_slots"))
				resolve_cache_slots = ltx->r_u32(CFG_SECTION, "resolve_cache_slots");
			if (ltx->line_exist(CFG_SECTION, "alpha_ref")) alpha_ref = ltx->r_float(CFG_SECTION, "alpha_ref");
		}
		xr_delete(ltx);

		clamp(ray_extend, 0.f, 5.f);
		clamp(foliage_k, 0.f, 100.f);
		clamp(canopy_k, 0.f, 100.f);
		clamp(atmosphere_k, 0.f, 100.f);
		clamp(max_slab_h, 0.f, 64.f);
		// A zero budget would turn every grass slot permanently opaque; a cache smaller than the slots
		// one long ray crosses would evict its own working set.
		clamp(resolve_budget, 1u, 4096u);
		clamp(resolve_cache_slots, 1024u, 262144u);
		clamp(alpha_ref, 0.f, 1.f);

		Msg("~ [perception] occlusion model: foliage_k=%.2f canopy_k=%.2f atmosphere_k=%.2f extend=%.2f max_slab=%.2f alpha_ref=%.3f",
		    foliage_k, canopy_k, atmosphere_k, ray_extend, max_slab_h, alpha_ref);
		Msg("~ [perception]   grass resolve budget %u slot(s)/frame, cache %u slot(s)", resolve_budget,
		    resolve_cache_slots);
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
		if (tau <= 0.f)
			return 1.f;

		return expf(-canopy_k * tau);
	}

	float occlusion_model::atmosphere_transmittance(float t) const
	{
		if (atmosphere_k <= 0.f || t >= 1.f)
			return 1.f;

		return powf(_max(t, 0.f), atmosphere_k);
	}
}
