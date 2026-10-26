#pragma once

#include "vision_sampler.h"

class CObject;

namespace perception
{
	// [vision_feed] and [vision_sampler] in creatures\perception.ltx, read at level load. The console
	// writes enabled, visible_min and rate_power live until the next load.
	struct feed_config
	{
		int enabled;          // NPC detection uses the feed; 0 = v1
		float visible_min;    // least v_p any body point needs for the target to be seen
		float rate_power;     // rate = Lua rate * visibility ^ rate_power
		u32 points;           // rays per pair per vision tick (FEED-DESIGN 2.1)
	};

	struct feed_verdict
	{
		bool seen;
		float visibility;     // FEED-DESIGN 2: weighted mean v_p, else the sticky point's
		float best;           // max v_p
		float factor;         // the rate factor, visibility ^ rate_power
		u16 bone;             // the point the NPC looks at (cp_LAST)
		Fvector local;        // bone-local
		u16 current;          // current rotating records
		occluder_class limiting;
		u8 degraded;
	};

	// The v1 adapter for one observer (FEED-DESIGN 3.1): samples creature targets and judges them by
	// the FEED-DESIGN 2 rule.
	class vision_feed
	{
	public:
		explicit vision_feed(s32 seed);

		static void on_level_load();
		static const feed_config& config() { return s_config; }
		static feed_config& tunables() { return s_config; }
		static bool enabled() { return 0 != s_config.enabled; }
		static void register_console_commands();

		// False when the feed does not judge the target (not a creature, no pickable shape): the caller
		// traces it the v1 way.
		bool judge(CObject* observer, const Fvector& eye, const CObject* target, float threshold, u32 now,
		           feed_verdict& out, trace_listener* listener = 0);

		// 1 for a target the feed has not judged.
		float rate_factor(u16 target_id) const;

		void retain_only(const xr_vector<u16>& target_ids);
		void clear();

		const vision_sampler& sampler() const { return m_sampler; }

	private:
		struct factor
		{
			u16 id;
			float value;
		};

		vision_sampler m_sampler;
		xr_vector<factor> m_factors;

		static feed_config s_config;
	};
}
