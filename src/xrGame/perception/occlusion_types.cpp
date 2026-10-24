#include "stdafx.h"
#include "occlusion_types.h"

namespace perception
{
	LPCSTR occluder_class_name(occluder_class c)
	{
		switch (c)
		{
		case occ_hard: return "hard";
		case occ_soft: return "soft";
		case occ_creature: return "creature";
		case occ_foliage: return "foliage";
		case occ_canopy: return "canopy";
		case occ_atmosphere: return "atmosphere";
		default: return "?";
		}
	}

	LPCSTR degraded_reason_name(u32 bit)
	{
		switch (bit)
		{
		case deg_foliage_unresolved: return "grass-unresolved";
		case deg_canopy_incomplete: return "canopy-incomplete";
		case deg_canopy_overcount: return "canopy-overcount";
		case deg_atmo_unmeasured: return "fog-unmeasured";
		default: return "?";
		}
	}

	bool segment::make(const Fvector& a, const Fvector& b, segment& out)
	{
		if (!_valid(a) || !_valid(b))
			return false;

		out.from = a;
		out.to = b;
		out.dir.sub(b, a);
		out.length = out.dir.magnitude();
		if (out.length < EPS_L)
			return false;

		out.dir.div(out.length);
		return true;
	}

	collide::ray_defs segment::to_ray_defs(u32 flags, collide::rq_target tgt, float extend) const
	{
		return collide::ray_defs(from, dir, length + extend, flags, tgt);
	}

	void occlusion_result::reset()
	{
		for (u32 i = 0; i < occ_class_count; ++i)
			channel[i] = 1.f;
		transmittance = 1.f;
		dominant = occ_hard;
		dominant_material = 0;
		foliage_tau = 0.f;
		canopy_tau = 0.f;
		canopy_cells = 0;
		deferred_hits = 0;
		deferred_t = 1.f;
		fallback_hits = 0;
		foliage_unresolved = 0;
		degraded = 0;
		self_hits = 0;
		self_range = flt_max;
		hit_count = 0;
		from_cache_depth = false;
		from_cache_geom = false;
	}

	void occlusion_result::finish()
	{
		transmittance = 1.f;
		dominant = occ_hard;

		float worst = flt_max;
		for (u32 i = 0; i < occ_class_count; ++i)
		{
			clamp(channel[i], 0.f, 1.f);
			transmittance *= channel[i];
			if (channel[i] < worst)
			{
				worst = channel[i];
				dominant = occluder_class(i);
			}
		}

		// dominant_material names the worst single GEOMETRY hit. When the dominant channel is not a
		// geometry one it belongs to a different answer entirely, and reporting "dominant foliage
		// (materials\bush)" invites exactly the wrong reading.
		if (occ_foliage == dominant || occ_canopy == dominant || occ_atmosphere == dominant)
			dominant_material = 0;
	}
}
