#pragma once

#include "../../build_config_defines.h"

#ifdef PERCEPTION_DEBUG

#include "vision_feed.h"

class CCustomMonster;

namespace perception
{
	namespace shadow
	{
		// Scoped around v1's own trace in CCustomMonster::eye_pp_s2: construction times v1, and
		// destruction, which runs once v1 has traced, samples the same pairs on the same eye.
		class tick
		{
		public:
			tick(CCustomMonster& observer, float dt);
			~tick();

		private:
			CCustomMonster& m_observer;
			float m_dt;
			CTimer m_timer;
		};

		enum mode
		{
			mode_off = 0,
			mode_actor,
			mode_all
		};

		// Scoped around one feed judgement in CCustomMonster::feel_vision_judge: times it, and for an
		// explained pair records its traces for the explain view and the ray log.
		class feed_pair
		{
		public:
			feed_pair(CCustomMonster& observer, const CObject* target);
			~feed_pair();

			trace_listener* listener() const { return m_listener; }
			void judged(const feed_verdict& v);

		private:
			CCustomMonster& m_observer;
			const CObject* m_target;
			trace_listener* m_listener; // null unless the pair is explained
			bool m_was_recording;
			bool m_log;
			CTimer m_timer;
		};

		// The rate get_visible_value applies to an explained pair, logged at the ray-log interval.
		void feed_rate(const CCustomMonster& observer, const CObject* target, float lua_rate, float factor);

		extern int g_mode; // perception_shadow

		LPCSTR mode_name(int mode);

		void on_level_load();
		void on_level_unload();

		void report();
		void reset();

		// Adds the explained pairs' records to the debug sink. Called every frame, so the view stays
		// up between vision ticks.
		void draw_explained();
	}
}

#endif
