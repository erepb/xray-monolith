#pragma once

#include "occlusion_types.h"

class CObject;
class IKinematics;

namespace perception
{
	struct sample_slot
	{
		u16 bone;
		float weight; // shape surface area, v1's own weight
	};

	struct slot_record
	{
		Fvector eye;                      // world, the origin of the last full trace
		Fvector point;                    // world, where the last full trace ended
		Fvector local;                    // the same point, bone-local
		static_half level;                // of the last full trace
		u32 refreshes;                    // since the last full trace
		float channel[occ_class_count];
		float t;                          // product of channel[]
		float t_v1;                       // occlusion_result::v1_transmittance()
		float v;                          // the point's visibility under the feed rule (FEED-DESIGN 2)
		occluder_class dominant;
		u8 degraded;
		bool self_occluded;               // the target's own body lies between the eye and the point
		u32 time;                         // Device.dwTimeGlobal; 0 = no current record
		u32 visits;
	};

	// Why a visit traced in full rather than refreshing, first matching reason in this order.
	enum full_reason
	{
		full_new_point = 0,  // no record yet, or the rotation drew a fresh point
		full_not_reusable,   // the last full trace left nothing reusable (a creature in the way, the budget)
		full_eye_moved,
		full_point_moved,
		full_visit_limit,    // refresh_visits reached
		full_reason_count
	};

	struct sample_counts
	{
		u32 full[full_reason_count];
		u32 refreshed;
		u32 skipped_static;               // still, behind its own static geometry: no ray at all
		u32 rerolls_best;                 // sticky point taken over from a visible rotating record
		u32 rerolls_fresh;                // sticky point drawn by v1's rule
	};

	struct pair_summary
	{
		// Over the rotating records, every age.
		float weight_total;
		float coverage;                   // weighted share with t >= clear_threshold
		float coverage_v1;                // same, on t_v1
		float coverage_v1_self;           // same, and not self-occluded: v1's own rules in full
		float self_share;                 // weighted share self-occluded
		float transmittance;              // weighted mean t
		occluder_class limiting;          // argmin of the weighted per-class mean; occ_class_count = none (every class clear)
		u8 degraded;                      // OR over records
		u32 oldest_age, newest_age;       // ms
		u16 slots;                        // visible slots now
		u16 sampled;                      // visible slots holding a record
		u32 visit_spread;                 // max - min visits over visible slots

		// FEED-DESIGN 2.1, from the sticky point and the current rotating records.
		bool has_sticky;
		bool seen;                        // best >= visible_min
		float best;                       // max v
		float visibility;                 // weighted mean v over current rotating records, else the sticky v
		u16 current;                      // current rotating records
		occluder_class sticky_dominant;   // the sticky point's lowest channel; occ_class_count = every class clear

		// Where the observer looks (FEED-DESIGN 2.1 cp_LAST): the sticky point while it passes, else the
		// best passing current rotating record, else the sticky point.
		bool has_point;
		u16 point_bone;
		Fvector point_local;
	};

	// FEED-DESIGN 2: a point's visibility from its channels, the materials gated at clear_threshold.
	float point_visibility(const float channel[occ_class_count], float clear_threshold);

	// A point fails when it is under visible_min, or blocked outright when visible_min is 0.
	bool point_fails(float v, float visible_min);

	// The lowest channel below 1; occ_class_count when every channel is clear.
	occluder_class lowest_channel(const float channel[occ_class_count]);

	// Called after every trace the sampler makes, while the query's per-trace debug detail is still
	// current.
	class trace_listener
	{
	public:
		virtual void on_trace(const segment& seg, u16 bone, const occlusion_result& r) = 0;
	};

	// Per-observer. Each call traces k rays to one target (FEED-DESIGN 2.1): one to the sticky point,
	// kept while it is visible and re-chosen at the next call when it is not, and k - 1 rotating over
	// the target's pickable bone shapes on a fixed cursor that advances whatever the outcome, so every
	// shape is sampled equally often. Each slot keeps only its latest record.
	class vision_sampler
	{
	public:
		explicit vision_sampler(s32 seed);

		// A record is refreshed against dynamic objects only while both ends of its sight line are
		// within current_eps metres of its last full trace; every refresh_visits-th visit is full.
		static void configure(float current_eps, u32 refresh_visits);

		// limits: the trace's early-outs; its reuse is the sampler's own. counts may be null.
		void sample(CObject* observer, const Fvector& eye, const CObject* target, u32 k, u32 now,
		            const trace_options& limits, sample_counts* counts, trace_listener* listener);

		// On false, out.slots still counts the visible slots (0 = no usable shape).
		bool summary(const CObject* target, const Fvector& eye, float clear_threshold, float visible_min, u32 now,
		             pair_summary& out) const;

		// Null when the target has no pair.
		const xr_vector<slot_record>* records(u16 target_id) const;
		const slot_record* sticky(u16 target_id) const;

		void retain_only(const xr_vector<u16>& target_ids);
		void clear() { m_pairs.clear(); }

	private:
		struct pair_state
		{
			u16 id;
			const CObject* object;        // identity only; an id reused by another object resets
			const void* visual;           // slots rebuilt when the visual changes
			xr_vector<sample_slot> slots;
			xr_vector<slot_record> records;
			u32 cursor;
			u64 bones_visible;            // visits restart when this changes
			slot_record sticky;
			u32 sticky_slot;
			bool sticky_set;
			Fvector block_tri[3];         // the last static triangle that blocked a full trace of this pair
			bool has_block_tri;
		};

		pair_state* find(u16 id);
		const pair_state* find(u16 id) const;
		pair_state& acquire(const CObject* target);
		static void build_slots(const CObject* target, pair_state& p);

		bool current(const CObject* target, const slot_record& rec, u16 bone, const Fvector& eye) const;
		void draw_sticky(pair_state& p, IKinematics* K);
		void reroll_sticky(pair_state& p, IKinematics* K, const CObject* target, const Fvector& eye, float visible_min,
		                   sample_counts* counts);
		bool visit(pair_state& p, CObject* observer, const Fvector& eye, const CObject* target, IKinematics* K,
		           u16 bone, slot_record& rec, bool keep_point, u32 now, const trace_options& limits,
		           sample_counts* counts, trace_listener* listener);

		xr_vector<pair_state> m_pairs;
		CRandom m_rnd;

		static float s_current_eps;
		static u32 s_refresh_visits;
	};
}
