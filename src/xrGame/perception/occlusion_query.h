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

	// The shared occlusion primitive. Not a percept: the firing path will call the same service with
	// its own origin, rate and threshold, so neither system inherits the other's cadence.
	class occlusion_query
	{
	public:
		static occlusion_query& instance();

		void on_level_load();
		void on_level_unload();
		bool ready() const { return m_ready; }

		bool trace(const segment& seg, u32 rq_flags, CObject* ignore, occlusion_result& out) const;

		// Prefix/tail form. `head` is traced once and reused across every tail sharing it; this, not
		// the cross-frame cache, is the primary efficiency mechanism (STAGE2-DESIGN 7.3).
		bool trace_head(const segment& head, u32 rq_flags, CObject* ignore,
		                occlusion_result& out) const;
		bool trace_tail(const occlusion_result& head_result, const segment& tail, u32 rq_flags,
		                CObject* ignore, occlusion_result& out) const;

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
		mutable xr_vector<hit_mark> last_hits;

		// Cached-vs-traced static geometry error. Free to collect: this stage traces both anyway.
		mutable u32 validate_n = 0;
		mutable float validate_sum = 0.f;
		mutable float validate_max = 0.f;
		void reset_validation() const { validate_n = 0; validate_sum = validate_max = 0.f; }
#endif

	private:
		bool trace_impl(const segment& seg, u32 rq_flags, CObject* ignore,
		                occlusion_result& out) const;

		mutable collide::rq_results m_hits;
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
