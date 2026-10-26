#pragma once

#include "occlusion_types.h"
#include "occluder_table.h"
#include "occlusion_model.h"
#include "static_occlusion_cache.h"
#include "foliage_oracle.h"

class CObject;

namespace perception
{
	// R1: the body of CVisualMemoryManager::feel_vision_mtl_transp, which uses no member state and is
	// reached through two virtual hops that no subclass overrides. That member now delegates here, so
	// v1 and v2 cannot drift - which matters because Stage 3 compares them directly.
	float material_transmittance(CObject* O, u32 element);

	// Game material index of the surface a hit landed on.
	u16 material_of(const collide::rq_result& h);

	// The test callback every vision ray is cast with (objects flagged STYPE_FEELVISIONIGNORE are not
	// tested); null when the engine has no such flag.
	collide::test_callback* vision_test_callback();

	// An AI-visible object on a segment, as v1's second pass finds it (Feel_Vision.cpp:260-295).
	// Creatures are not obstacles, so RayQuery's object pass never returns them; this is where they
	// block.
	struct body_hit
	{
		CObject* O;   // null for a spatial that is not a CObject
		float range;  // metres from the segment start
		bool sphere;  // no collision model: counted on its bounding sphere, as v1 counts it
		bool drawn;   // has a visual and is rendered
	};

	// Every AI-visible object within `range` along seg, except `ignore`, `target` and objects flagged
	// for vision to ignore, near to far. drawn_only also drops objects nothing draws: "visible for AI"
	// is a spawn flag, and invisible anomaly zones carry it.
	void ai_visible_hits(const segment& seg, float range, const CObject* ignore, const CObject* target,
	                     bool drawn_only, xr_vector<body_hit>& out);

	// The shared occlusion primitive. Not a percept: the firing path will call the same service with
	// its own origin, rate and threshold, so neither system inherits the other's cadence.
	class occlusion_query
	{
	public:
		static occlusion_query& instance();

		void on_level_load();
		void on_level_unload();
		bool ready() const { return m_ready; }

		// `ignore` is never hit (the observer). Hits on `target` are skipped and reported in
		// self_hits/self_range; either may be null.
		bool trace(const segment& seg, u32 rq_flags, CObject* ignore, const CObject* target,
		           occlusion_result& out, const trace_options& opt = trace_options()) const;

		const occlusion_model& model() const { return m_model; }

		// Writable view, for the console knobs. The address is stable - this is a singleton - and a
		// level load re-reads the ltx over the top, which keeps the config file authoritative and
		// makes any console value explicitly a within-session experiment.
		occlusion_model& tunables() { return m_model; }
		const occluder_table& table() const { return m_table; }
		static_occlusion_cache& cache() const { return m_cache; }

#ifdef PERCEPTION_DEBUG
		// Detail of the last trace, for the debug sink. Only populated when `record` is set, which
		// also forces the real foliage march instead of a cached depth - the view must show what is
		// actually there.
		mutable bool record = false;
		// Kept apart rather than merged: the probe reports grass coverage and canopy coverage as
		// separate figures, and a single list could not tell which field produced a crossing.
		mutable xr_vector<veg_chord> last_chords;
		mutable xr_vector<veg_chord> last_canopy;
		mutable xr_vector<canopy_hit> last_canopy_hits;
		mutable xr_vector<hit_mark> last_hits;

		// What concealment costs per trace, and how much of it the early-outs spare.
		struct march_stats
		{
			u32 traces;                     // reached the canopy march
			double canopy_us;
			u32 grass_traces, grass_skipped; // grass marched / spared because fog times canopy hid the point
			double grass_us, grass_resolve_us;
			u32 unresolved;
			u32 fog_skipped, material_skipped;
			u32 t_start;
			void reset() { ZeroMemory(this, sizeof(*this)); t_start = Device.dwTimeGlobal; }
		};
		mutable march_stats march_cost = {};

		// What the blocking-triangle shortcut and the bodies early-out spare, and what the query costs.
		struct ray_stats
		{
			u32 tri_hits;                   // traces answered by options.block_tri
			u32 tri_checked, tri_wrong;     // of them, traced in full anyway; and found not blocked
			u32 queries;                    // RayQuery calls
			double query_us;
			u32 bodies_run, bodies_skipped; // bodies passes run / spared behind a material block
			double bodies_us;
			void reset() { ZeroMemory(this, sizeof(*this)); }
		};
		mutable ray_stats ray_cost = {};

		// Cached-vs-traced static geometry error. Free to collect: this stage traces both anyway.
		mutable u32 validate_n = 0;
		mutable float validate_sum = 0.f;
		mutable float validate_max = 0.f;
		void reset_validation() const { validate_n = 0; validate_sum = validate_max = 0.f; }
#endif

	private:
		mutable collide::rq_results m_hits;
		mutable xr_vector<body_hit> m_bodies;
		mutable static_occlusion_cache m_cache;
		occluder_table m_table;
		occlusion_model m_model;
		bool m_ready = false;
	};

	// Level hooks for the whole occlusion half of the module, so Level.cpp/Level_load.cpp gain one
	// call each rather than one per component.
	namespace occlusion
	{
		void on_level_load();
		void on_level_unload();
	}
}
