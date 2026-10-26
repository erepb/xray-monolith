#pragma once

#include "../../build_config_defines.h"
#include "../../xrCDB/xr_collide_defs.h"

namespace perception
{
	enum occluder_class
	{
		occ_hard = 0,
		occ_soft,
		occ_creature,
		occ_foliage,
		occ_canopy,
		occ_atmosphere,
		occ_class_count
	};

	LPCSTR occluder_class_name(occluder_class c);

	// Set by each fallback where it fires; a result with none set is measured on every channel.
	enum degraded_reason
	{
		deg_foliage_unresolved = 1 << 0, // grass slots past the resolve budget, counted opaque
		deg_canopy_incomplete = 1 << 1,  // the field skipped leaf geometry; proxy hulls also count
		deg_canopy_overcount = 1 << 2,   // crossed a field holding crowns baked with every LOD summed
		deg_atmo_unmeasured = 1 << 3,    // the fog probe produced no curve; analytic stand-in
		deg_reason_count = 4
	};

	LPCSTR degraded_reason_name(u32 bit);

	// The one place a pair of points becomes a ray. It ends exactly at `to`, where v1's ray ends on
	// level geometry.
	struct segment
	{
		Fvector from;
		Fvector to;
		Fvector dir; // normalised
		float length;

		// The module's only guard against non-finite world coordinates: the foliage march feeds
		// caller-supplied floats straight to iFloor, whose behaviour on NaN/Inf is UB. Every segment
		// is born here, so that path cannot receive garbage from this module.
		static bool make(const Fvector& a, const Fvector& b, segment& out);

		collide::ray_defs to_ray_defs(u32 flags, collide::rq_target tgt) const;
	};

	// Where a ray met an occluder. Lives here rather than in the debug header so occlusion_query can
	// record them without depending on the debug module.
	struct hit_mark
	{
		Fvector pos;
		occluder_class cls;
		float t;
	};

	// One vegetation crossing, for the debug view. Shared by the grass and canopy marches so the sink
	// and the colour policy have a single form to handle.
	struct veg_chord
	{
		Fvector enter, exit;
		float blocked; // 0..1 fraction of this crossing that is opaque

		// The crossed grass tuft (bounding cylinder) or crown (half the larger side of its world box),
		// world space.
		Fvector origin = {0.f, 0.f, 0.f};
		float radius = 0.f;
		float y_lo = 0.f, y_hi = 0.f;
		float core_radius = 0.f;            // 90 % of the drawn opaque area lies inside
		float core_top = 0.f;               // 95 % lies below
		float scale = 0.f;
		float tau = 0.f;
		u16 model = 0xffff;
		u32 crown = u32(-1);                // canopy only: the crossed crown
		u16 hits = 0;                       // canopy only: cards pierced
		bool inside = false;                // canopy only: the eye is inside this crown
	};

	// One leaf card a canopy ray pierced, for the probe.
	struct canopy_hit
	{
		u32 crown;
		float t;         // metres from the eye
		float footprint; // metres, the sight line's width at t
		float level;     // coverage pyramid level read, 0 = base
		float coverage;  // 0..1, leaf share of the footprint
		bool near_plane; // nearer the eye than the renderer draws; not counted
	};

	// What a trace found that cannot change while neither end of its segment moves: static geometry,
	// grass and canopy. Creatures, doors and other dynamic objects are not in it.
	struct static_half
	{
		float geom[occ_class_count]; // static geometry hits per class
		float deferred_t;            // static proxy hulls the canopy channel replaces
		u16 deferred_hits;
		u16 fallback_hits;
		float foliage_tau;
		float canopy_tau;
		u16 canopy_crowns;
		u8 degraded;                 // canopy reasons only
		bool canopy_marched;
		bool grass_marched;          // false when fog, or fog times canopy, already hid the point
		float static_materials;      // hard x soft x creature of the static hits alone
		bool reusable;               // complete enough to stand in for a full trace (FEED-DESIGN 3.6)
	};

	struct trace_options
	{
		// Grass and canopy are not marched once the material channels alone fall below this; 0 always
		// marches.
		float clear_threshold = 0.f;
		// Canopy is not marched once fog alone falls below this, nor grass once fog times canopy does;
		// 0 always marches.
		float visible_min = 0.f;
		// Only the material channels are wanted: grass and canopy are never marched.
		bool materials_only = false;
		// The static half of an earlier full trace of this segment. Only dynamic objects are traced;
		// the rest comes from it.
		const static_half* reuse = 0;
		// A static triangle that blocked an earlier trace (occlusion_result::blocker). A segment that
		// crosses its front face is hidden without a ray. Ignored by a reuse.
		const Fvector* block_tri = 0;
	};

	// A point on the near surface is hit by its own shape at the end of the segment, give or take
	// float noise; only a hit clearly nearer than that means the body is in front of the point.
	static const float SELF_EPS = 0.05f;

	struct occlusion_result
	{
		float channel[occ_class_count]; // per-class transmittance
		float transmittance;            // == product of channel[]
		occluder_class dominant;        // argmin channel[]
		shared_str dominant_material;   // worst single hit; empty for foliage
		float foliage_tau;              // optical depth of grass on the path, before the transfer function
		float canopy_tau;               // optical depth of tree canopy, likewise

		// Crowns whose cards the ray pierced. Unlike the debug chord list this is counted whether or
		// not the debug view is recording.
		u16 canopy_crowns;

		// Geometry hits skipped because a material maps to occ_canopy: the canopy field already
		// accounts for that vegetation, so counting the proxy hull too would double it. Reported so
		// the probe can say the hull was seen and deliberately ignored.
		u16 deferred_hits;

		// Product of the transmittances of those skipped proxy hits, kept so a v1-comparable
		// transmittance stays reconstructable: Stage 3 compares v2 against v1 directly, and v1 counts
		// these hits.
		float deferred_t;

		// Canopy-class hits that were counted as soft because there was no complete canopy field to
		// defer them to.
		u16 fallback_hits;

		// Slots the march could not resolve within the frame budget and therefore counted as fully
		// occluding. Non-zero means this result is a deliberate over-estimate that will settle within
		// a frame or two - it must not be cached, and the debug view flags it.
		u16 foliage_unresolved;

		u8 degraded; // degraded_reason bits

		// Hits on the target itself, skipped: a point is hidden only when something else is in the
		// way. self_range is the nearest of them along the ray, flt_max when none - nearer than the
		// segment's end means the point faces away from the eye.
		u16 self_hits;
		float self_range;

		// The target's own body lies between the eye and the point; `length` is the segment's.
		bool self_occluded(float length) const
		{
			return self_hits && (self_range < length - SELF_EPS);
		}

		// AI-visible objects on the segment (v1's second pass), each blocking outright. sphere_hits of
		// them have no collision model and are counted on their bounding sphere, as v1 counts them.
		u16 body_hits;
		u16 sphere_hits;

		// Filled by every full trace; a reuse copies it through.
		static_half level;
		bool refreshed;            // traced from options.reuse
		bool concealment_skipped;  // materials or fog already hide the point, so grass and canopy were not marched
		bool tri_blocked;          // hidden by options.block_tri, no ray cast

		// The nearest level triangle on the segment that blocks on its own, vertices in winding order;
		// never a vegetation proxy hull.
		Fvector blocker[3];
		bool has_blocker;

		u16 hit_count;
		// Kept separate because the two halves of the cache have different tolerances, and because
		// the debug layer splits hit rate by whether the target came out occluded - the reroll at
		// Feel_Vision.cpp:302 predicts those two populations differ.
		bool from_cache_depth;
		bool from_cache_geom;

		// v1 counts the vegetation proxy hits that the canopy channel replaces, so a v1-comparable
		// value needs them back: deferred_t carries their product. In the fallback case they are
		// already inside channel[occ_soft] and deferred_t is 1. Stage 3 compares v2 against v1
		// directly, and Stage 4's exit criterion needs a reference that reproduces v1.
		float v1_transmittance() const
		{
			return channel[occ_hard] * channel[occ_soft] * channel[occ_creature] * deferred_t;
		}

		void reset();

		// Collapses the channels into transmittance and dominant.
		void finish();
	};
}
