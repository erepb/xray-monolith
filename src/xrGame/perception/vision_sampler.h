#pragma once

#include "occlusion_types.h"

class CObject;

namespace perception
{
	struct sample_slot
	{
		u16 bone;
		float weight; // shape surface area, v1's own weight
	};

	struct slot_record
	{
		Fvector eye;                      // world, the origin it was traced from
		Fvector point;                    // world, where it was sampled
		float channel[occ_class_count];
		float t;                          // product of channel[]
		float t_v1;                       // occlusion_result::v1_transmittance()
		occluder_class dominant;
		u8 degraded;
		bool self_occluded;               // the target's own body lies between the eye and the point
		u32 time;                         // Device.dwTimeGlobal; 0 = no current record
		u32 visits;
	};

	struct pair_summary
	{
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
	};

	// Called after every trace the sampler makes, while the query's per-trace debug detail is still
	// current.
	class trace_listener
	{
	public:
		virtual void on_trace(const occlusion_result& r) = 0;
	};

	// Per-observer. Each call traces k body points of one target, rotating over the target's pickable
	// bone shapes on a fixed cursor that advances whatever the outcome, so every shape is sampled
	// equally often. Each slot keeps only its latest record.
	class vision_sampler
	{
	public:
		explicit vision_sampler(s32 seed);

		void sample(CObject* observer, const Fvector& eye, const CObject* target, u32 k, u32 now,
		            trace_listener* listener);

		// On false, out.slots still counts the visible slots (0 = no usable shape).
		bool summary(const CObject* target, float clear_threshold, u32 now, pair_summary& out) const;

		// Null when the target has no pair.
		const xr_vector<slot_record>* records(u16 target_id) const;

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
		};

		pair_state* find(u16 id);
		const pair_state* find(u16 id) const;
		pair_state& acquire(const CObject* target);
		static void build_slots(const CObject* target, pair_state& p);

		xr_vector<pair_state> m_pairs;
		CRandom m_rnd;
	};
}
