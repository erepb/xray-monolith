#pragma once

#include "../../build_config_defines.h"
#include "../../Include/xrRender/DetailInstances.h"
#include "foliage_profile.h"
#include "occlusion_types.h"

namespace perception
{
	// AI-oriented view of level foliage.
	//
	// Grass occlusion is resolved against the actual detail instances the renderer places: roads,
	// paths and trampled ground live within a 2 m slot, so nothing coarser than an instance can tell
	// them from the grass beside them.
	//
	// Instances come from IRender_interface::resolve_detail_slot, which runs the engine's own
	// placement for one slot. Placement is not a formula that stays put - it carries two user
	// sliders, a terrain-slope rejection and a terrain-align rotation - so it is called, never
	// reimplemented. See FOLIAGE-DISCRETE-DESIGN.
	class foliage_oracle
	{
	public:
		static foliage_oracle& instance();

		void on_level_load();
		void on_level_unload();
		bool ready() const { return m_ready; }

		// False when this renderer places no detail objects at all.
		bool renderer_places_detail() const { return m_renderer_places; }

		struct march_result
		{
			float tau;        // optical depth: -sum(ln T_i) over crossed instances
			u16 unresolved;   // slots that had to be assumed occluded, see march()
		};

		// Grass along a segment.
		//
		// Slots the ray crosses are resolved on demand within a per-frame budget. A slot that could
		// not be resolved this frame counts as FULLY OCCLUDING, not as clear: X-Ray's visual memory
		// latches, so a false sighting enters combat, persists and propagates to the squad and cannot
		// be taken back a frame later, while a missed one costs a frame and self-corrects.
		void march(const Fvector& from, const Fvector& to, march_result& out,
		           xr_vector<veg_chord>* chords = 0) const;

		// Per-frame budget reset, placement-change poll and eviction. Called from the level tick.
		void on_frame();

		// Changes whenever the placement the oracle resolved from changes; cached grass depth from an
		// earlier epoch is stale.
		u32 placement_epoch() const { return m_placement_epoch; }

		// Slots resolved per frame engine-wide, and resolved slots retained.
		void set_resolve_limits(u32 budget, u32 cache_slots)
		{
			m_resolve_budget = budget;
			m_cache_max = cache_slots;
		}

		// The detail shader's alpha-test cutoff, 0..1. Read by the next on_level_load's bake.
		void set_alpha_ref(float a) { m_alpha_ref = a; }

		// Vertical band grass could occupy at a world XZ, from the baked slot bounds plus the tallest
		// model the slot references. Cheap reject before any slot is resolved. false = no foliage ids.
		bool band_at(float x, float z, float& y_lo, float& y_hi) const;

#ifdef PERCEPTION_DEBUG
		struct stats
		{
			u32 resolved_slots;   // slots currently cached
			u32 resolve_requests; // total resolves performed
			float resolve_us;     // total microseconds spent inside resolve_detail_slot
			u32 budget_denied;    // slots that hit the frame budget and were assumed occluded
			u32 evicted;          // resolved slots dropped to keep the cache under its cap
			u32 instances;        // instances currently cached

			u32 marches;          // march() calls
			u32 slots_walked;     // slots crossed that hold any foliage ids at all
			u32 slots_above;      // ...of those, rejected because the ray passed outside the band
			u32 slots_entered;    // ...of those, actually intersected against instances
		};

		// Counters only - never the cache itself, so measuring never changes what is measured.
		void reset_stats() const
		{
			const u32 slots = u32(m_resolved.size());
			const u32 inst = m_instance_count;
			ZeroMemory(&m_stats, sizeof(m_stats));
			m_stats.resolved_slots = slots;
			m_stats.instances = inst;
		}

		const stats& get_stats() const
		{
			m_stats.resolved_slots = u32(m_resolved.size());
			m_stats.instances = m_instance_count;
			return m_stats;
		}
#endif

	private:
		// The slot grid stays as level.details holds it: bounds for the pre-reject and the model ids
		// that decide the band's top. 4 bytes per 2 m slot, level-wide.
		struct cell
		{
			u8 tallest;         // 10 cm units, highest model top at the largest placement scale, before the height slider; 0 = no foliage
			u8 slab_h;          // 10 cm units, terrain relief within the slot
			u16 slab_base : 12; // 20 cm units, world = base*0.2 - 200
			u16 pad : 4;
		};

		static_assert(sizeof(cell) == 4, "cell must stay 4 bytes - one per 2 m slot level-wide");

		struct resolved
		{
			xr_vector<detail_instance> items;
			u32 last_used;
		};

		typedef xr_map<u32, resolved> resolved_map; // key = linear slot index

		// Resolve-or-refuse. Returns null when the slot has not been resolved and the frame budget is
		// spent; the caller must then treat the slot as occluding.
		const resolved* slot_instances(int gx, int gz) const;

		// Intersects one resolved slot's instances with the ray and folds them into `out`.
		void accumulate_slot(const resolved& r, const Fvector& from, const Fvector& dir, float len,
		                     march_result& out, xr_vector<veg_chord>* chords) const;

		void evict_if_needed() const;

		// Metres above the slot's base that its grass can reach.
		float band_height(const cell& c) const;

		// The cell at grid column (gx, gz), slot coordinates plus the grid offset; null outside the grid.
		const cell* cell_at(int gx, int gz) const;
		static float slab_floor(const cell& c) { return float(c.slab_base) * 0.2f - 200.f; }

		xr_vector<cell> m_field;
		xr_vector<foliage_model_profile> m_profiles;
		u32 m_size_x = 0;
		u32 m_size_z = 0;
		int m_offs_x = 0;
		int m_offs_z = 0;
		u32 m_resolve_budget = 0;
		u32 m_cache_max = 0;
		float m_alpha_ref = 200.f / 255.f;
		bool m_ready = false;
		bool m_renderer_places = false;
		u32 m_placement_epoch = 0;
		u32 m_frame = 0;

		// Last renderer placement state seen, see on_frame().
		u32 m_detail_gen = 0;
		float m_detail_height = 1.f;
		float m_detail_slope = 1.f;
		bool m_details_on = true;

		mutable resolved_map m_resolved;
		mutable u32 m_instance_count = 0;
		mutable u32 m_budget_left = 0;
#ifdef PERCEPTION_DEBUG
		mutable stats m_stats;
#endif
	};
}
