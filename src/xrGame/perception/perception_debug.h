#pragma once

#include "../../build_config_defines.h"

#ifdef PERCEPTION_DEBUG

#	include "occlusion_types.h"

#	define PDBG_LOG(...)		::Msg(__VA_ARGS__)
#	define PDBG_SCOPE(name)		::perception::debug::scope_timer _pdbg_scope_(name)
#	define PDBG_ONLY(code)		code

namespace perception
{
	namespace debug
	{
		struct scope_timer
		{
			CTimer timer;
			LPCSTR name;

			scope_timer(LPCSTR n);
			~scope_timer();
		};

		struct ray_record
		{
			Fvector from, to;
			float transmittance;
			occluder_class dominant;
			float freshness; // 1 = traced now, falling toward 0 with age; dims the end box
		};

		// v1's own sample point for the same pair, so v1 and v2 read side by side on one body.
		struct v1_point_record
		{
			Fvector pos;
			bool visible;
		};

		typedef veg_chord chord_record;
		typedef hit_mark point_record;

		// Records are semantic, not geometric: a ray carries its transmittance, never a colour, so
		// the legend in a screenshot and the numbers in the log cannot drift apart.
		class sink
		{
		public:
			static sink& instance();

			// Producer side, driven from CLevel::OnFrame.
			void begin_frame();
			void add(const ray_record& r);
			void add(const chord_record& c);
			void add(const point_record& p);
			void add(const v1_point_record& p);

			// Sticky set: survives frames until explicitly cleared. This is what makes a one-shot
			// probe inspectable - trace once, then fly around and look at it.
			void begin_sticky();
			void add_sticky(const ray_record& r);
			void add_sticky(const chord_record& c);
			void add_sticky(const point_record& p);
			void clear_sticky();

			// Consumer side, driven from CLevel::OnRender.
			void flush();

			void clear();

		private:
			struct buffer
			{
				xr_vector<ray_record> rays;
				xr_vector<chord_record> chords;
				xr_vector<point_record> points;
				xr_vector<v1_point_record> v1_points;

				void clear();
				void swap(buffer& o);
			};

			static void draw(const buffer& b);

			buffer m_build, m_draw, m_sticky;
			bool m_ticked = false;
		};

		// Hit rate has to be split by whether the target came out occluded: cp_LP rerolls on every
		// failed evaluation (Feel_Vision.cpp:302), so the two populations are predicted to differ and
		// a blended number would hide exactly that.
		struct cache_split
		{
			u32 traces[2];
			u32 depth_hits[2];
			u32 geom_hits[2];
			u32 degraded[2];

			void reset();
			void note(bool occluded, const occlusion_result& r);
		};

		extern cache_split g_cache_split;

		// Space-separated degraded_reason names, or "-" when none is set.
		LPCSTR describe_degraded(u8 mask, string256& buf);

		// Cvars, owned here and written by the console commands.
		extern int g_draw;          // perception_dbg_draw
		extern int g_draw_tufts;    // perception_dbg_tufts
		extern int g_log;           // perception_dbg_log
		extern int g_log_interval;  // perception_dbg_log_interval, ms
		extern int g_explain;       // perception_explain, live view toggle
		extern u16 g_explain_id;    // 0xffff = all trackers

		void on_frame();
		void flush();
		void register_console_commands();
	}
}

#else

#	define PDBG_LOG(...)		((void)0)
#	define PDBG_SCOPE(name)		((void)0)
#	define PDBG_ONLY(code)

namespace perception
{
	namespace debug
	{
		IC void on_frame() {}
		IC void flush() {}
		IC void register_console_commands() {}
	}
}

#endif
