#pragma once

#include "../../build_config_defines.h"

#ifdef PERCEPTION_DEBUG

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
