#include "stdafx.h"
#include "perception_debug.h"

#ifdef PERCEPTION_DEBUG

#include "shadow_harness.h"
#include "../Level.h"
#include "../debug_renderer.h"

namespace perception
{
	namespace debug
	{
		int g_draw = 1;
		int g_draw_tufts = 0;
		int g_log = 0;
		int g_log_interval = 1000;
		int g_explain = 0;
		u16 g_explain_id = 0xffff;

		cache_split g_cache_split;

		scope_timer::scope_timer(LPCSTR n) : name(n)
		{
			timer.Start();
		}

		scope_timer::~scope_timer()
		{
			Msg("~ [perception] %-34s %6.1f ms", name, timer.GetElapsed_sec() * 1000.f);
		}

		void cache_split::reset()
		{
			ZeroMemory(traces, sizeof(traces));
			ZeroMemory(depth_hits, sizeof(depth_hits));
			ZeroMemory(geom_hits, sizeof(geom_hits));
			ZeroMemory(degraded, sizeof(degraded));
		}

		void cache_split::note(bool occluded, const occlusion_result& r)
		{
			const u32 i = occluded ? 1 : 0;
			++traces[i];
			if (r.from_cache_depth) ++depth_hits[i];
			if (r.from_cache_geom) ++geom_hits[i];
			if (r.degraded) ++degraded[i];
		}

		LPCSTR describe_degraded(u8 mask, string256& buf)
		{
			buf[0] = 0;
			for (u32 i = 0; i < deg_reason_count; ++i)
			{
				if (!(mask & (1u << i)))
					continue;
				if (buf[0])
					xr_strcat(buf, " ");
				xr_strcat(buf, degraded_reason_name(1u << i));
			}
			if (!buf[0])
				xr_strcpy(buf, "-");
			return buf;
		}

		// ---- colour policy: the only place a number becomes a colour -------------------------

		static IC u32 ray_colour(float t)
		{
			clamp(t, 0.f, 1.f);
			return D3DCOLOR_XRGB(int((1.f - t) * 255.f), int(t * 255.f), 0);
		}

		// One chord is now one crossed tuft, and `blocked` is already the 0..1 opaque fraction of it.
		static IC u32 chord_colour(float blocked)
		{
			clamp(blocked, 0.f, 1.f);
			const int v = int((0.35f + 0.65f * blocked) * 255.f);
			return D3DCOLOR_XRGB(v, v, 0);
		}

		static IC u32 dimmed(u32 c, float k)
		{
			clamp(k, 0.f, 1.f);
			return D3DCOLOR_XRGB(int(float(color_get_R(c)) * k), int(float(color_get_G(c)) * k),
			                     int(float(color_get_B(c)) * k));
		}

		static IC u32 class_colour(occluder_class c)
		{
			switch (c)
			{
			case occ_hard: return D3DCOLOR_XRGB(255, 40, 40);
			case occ_soft: return D3DCOLOR_XRGB(255, 200, 40);
			case occ_creature: return D3DCOLOR_XRGB(255, 40, 255);
			case occ_foliage: return D3DCOLOR_XRGB(40, 255, 120);
			case occ_canopy: return D3DCOLOR_XRGB(60, 200, 160);
			case occ_atmosphere: return D3DCOLOR_XRGB(120, 180, 255);
			default: return D3DCOLOR_XRGB(200, 200, 200);
			}
		}

		// ---- sink ----------------------------------------------------------------------------

		void sink::buffer::clear()
		{
			rays.clear();
			chords.clear();
			points.clear();
			v1_points.clear();
		}

		void sink::buffer::swap(buffer& o)
		{
			rays.swap(o.rays);
			chords.swap(o.chords);
			points.swap(o.points);
			v1_points.swap(o.v1_points);
		}

		sink& sink::instance()
		{
			static sink s_sink;
			return s_sink;
		}

		void sink::begin_frame()
		{
			m_build.clear();
			m_ticked = true;
		}

		void sink::add(const ray_record& r) { m_build.rays.push_back(r); }
		void sink::add(const chord_record& c) { m_build.chords.push_back(c); }
		void sink::add(const point_record& p) { m_build.points.push_back(p); }
		void sink::add(const v1_point_record& p) { m_build.v1_points.push_back(p); }

		void sink::begin_sticky() { m_sticky.clear(); }
		void sink::add_sticky(const ray_record& r) { m_sticky.rays.push_back(r); }
		void sink::add_sticky(const chord_record& c) { m_sticky.chords.push_back(c); }
		void sink::add_sticky(const point_record& p) { m_sticky.points.push_back(p); }
		void sink::clear_sticky() { m_sticky.clear(); }

		void sink::clear()
		{
			m_build.clear();
			m_draw.clear();
			m_sticky.clear();
			m_ticked = false;
		}

		void sink::flush()
		{
			if (m_ticked)
			{
				m_draw.swap(m_build);
				m_build.clear();
				m_ticked = false;
			}

			if (!g_draw)
				return;

			draw(m_draw);
			draw(m_sticky);
		}

		void sink::draw(const buffer& b)
		{
			CDebugRenderer& r = Level().debug_renderer();
			const float s = 0.05f;

			for (u32 i = 0; i < b.rays.size(); ++i)
			{
				const ray_record& x = b.rays[i];
				r.draw_line(Fidentity, x.from, x.to, ray_colour(x.transmittance), false);
				r.draw_aabb(x.from, s, s, s, D3DCOLOR_XRGB(60, 120, 255), false);
				r.draw_aabb(x.to, s, s, s, dimmed(class_colour(x.dominant), 0.25f + 0.75f * x.freshness), false);
			}

			// Drawn after the rays so the chord overlays the segment it belongs to.
			for (u32 i = 0; i < b.chords.size(); ++i)
			{
				const chord_record& x = b.chords[i];
				r.draw_line(Fidentity, x.enter, x.exit, chord_colour(x.blocked), false);
				if (g_draw_tufts && x.radius > 0.f)
				{
					Fvector c = x.origin;
					c.y = (x.y_lo + x.y_hi) * .5f;
					r.draw_aabb(c, x.radius, (x.y_hi - x.y_lo) * .5f, x.radius, dimmed(chord_colour(x.blocked), 0.4f), false);
					if (x.core_radius > 0.f && x.core_top > x.y_lo)
					{
						c.y = (x.y_lo + x.core_top) * .5f;
						r.draw_aabb(c, x.core_radius, (x.core_top - x.y_lo) * .5f, x.core_radius,
						            D3DCOLOR_XRGB(255, 140, 0), false);
					}
				}
			}

			for (u32 i = 0; i < b.points.size(); ++i)
			{
				const point_record& x = b.points[i];
				r.draw_aabb(x.pos, s * .6f, s * .6f, s * .6f, class_colour(x.cls), false);
			}

			for (u32 i = 0; i < b.v1_points.size(); ++i)
			{
				const v1_point_record& x = b.v1_points[i];
				r.draw_aabb(x.pos, s * 1.6f, s * 1.6f, s * 1.6f,
				            x.visible ? D3DCOLOR_XRGB(255, 255, 255) : D3DCOLOR_XRGB(70, 70, 70), false);
			}
		}

		void flush()
		{
			sink::instance().flush();
		}

		// ---- live view -------------------------------------------------------------------------

		void on_frame()
		{
			// CLevel::OnFrame still runs while paused in this engine - the fTimeDelta guard in
			// IGame_Level::OnFrame is commented out - so retention has to be explicit rather than a
			// consequence of the scheduler stopping.
			if (Device.Paused())
				return;

			sink::instance().begin_frame();

			if (g_explain && g_pGameLevel)
				shadow::draw_explained();
		}
	}
}

#endif
