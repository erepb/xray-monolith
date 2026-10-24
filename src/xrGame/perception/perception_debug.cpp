#include "stdafx.h"
#include "perception_debug.h"

#ifdef PERCEPTION_DEBUG

#include "occlusion_query.h"
#include "../Level.h"
#include "../Actor.h"
#include "../CustomMonster.h"
#include "../visual_memory_manager.h"
#include "../debug_renderer.h"

namespace perception
{
	namespace debug
	{
		int g_draw = 1;
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
		}

		void sink::buffer::swap(buffer& o)
		{
			rays.swap(o.rays);
			chords.swap(o.chords);
			points.swap(o.points);
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
				r.draw_aabb(x.to, s, s, s, class_colour(x.dominant), false);
			}

			// Drawn after the rays so the chord overlays the segment it belongs to.
			for (u32 i = 0; i < b.chords.size(); ++i)
			{
				const chord_record& x = b.chords[i];
				r.draw_line(Fidentity, x.enter, x.exit, chord_colour(x.blocked), false);
			}

			for (u32 i = 0; i < b.points.size(); ++i)
			{
				const point_record& x = b.points[i];
				r.draw_aabb(x.pos, s * .6f, s * .6f, s * .6f, class_colour(x.cls), false);
			}
		}

		void flush()
		{
			sink::instance().flush();
		}

		// ---- live view -------------------------------------------------------------------------

		static u32 s_next_log = 0;

		static void collect(CCustomMonster* m, const CObject* target, bool sticky)
		{
			occlusion_query& q = occlusion_query::instance();
			if (!q.ready() || !m || !target)
				return;

			Fvector to;
			bool found = false;
			for (u32 i = 0; i < m->feel_visible.size(); ++i)
			{
				if (m->feel_visible[i].O == target)
				{
					to = m->feel_visible[i].cp_LAST;
					found = true;
					break;
				}
			}
			if (!found)
				return;

			segment seg;
			if (!segment::make(m->eye_matrix.c, to, seg))
				return;

			q.record = (g_draw != 0);

			occlusion_result res;
			if (!q.trace(seg, CDB::OPT_CULL, m, res))
				return;

			// v1's own threshold, so the split matches the condition that actually rerolls cp_LP.
			float thr = 0.5f;
			CVisualMemoryManager* vm = m->visual_memory();
			if (vm)
				thr = vm->transparency_threshold();
			g_cache_split.note(res.transmittance < thr, res);

			sink& sk = sink::instance();

			ray_record rr;
			rr.from = seg.from;
			rr.to = seg.to;
			rr.transmittance = res.transmittance;
			rr.dominant = res.dominant;
			if (sticky) sk.add_sticky(rr); else sk.add(rr);

			for (u32 i = 0; i < q.last_chords.size(); ++i)
			{
				chord_record cr;
				cr.enter = q.last_chords[i].enter;
				cr.exit = q.last_chords[i].exit;
				cr.blocked = q.last_chords[i].blocked;
				if (sticky) sk.add_sticky(cr); else sk.add(cr);
			}

			for (u32 i = 0; i < q.last_canopy.size(); ++i)
			{
				chord_record cr;
				cr.enter = q.last_canopy[i].enter;
				cr.exit = q.last_canopy[i].exit;
				cr.blocked = q.last_canopy[i].blocked;
				if (sticky) sk.add_sticky(cr); else sk.add(cr);
			}

			for (u32 i = 0; i < q.last_hits.size(); ++i)
			{
				point_record pr;
				pr.pos = q.last_hits[i].pos;
				pr.cls = q.last_hits[i].cls;
				pr.t = q.last_hits[i].t;
				if (sticky) sk.add_sticky(pr); else sk.add(pr);
			}

			if (g_log && Device.dwTimeGlobal >= s_next_log)
			{
				string256 deg;
				Msg("~ [perception] %-24s T=%.3f  dom %-8s %-24s  tau %.2f  canopy tau %.2f  v1 T=%.3f  hits %u  degraded %s",
				    m->cName().c_str(), res.transmittance, occluder_class_name(res.dominant),
				    res.dominant_material.size() ? res.dominant_material.c_str() : "-",
				    res.foliage_tau, res.canopy_tau, res.v1_transmittance(), res.hit_count,
				    describe_degraded(res.degraded, deg));
			}
		}

		void on_frame()
		{
			// CLevel::OnFrame still runs while paused in this engine - the fTimeDelta guard in
			// IGame_Level::OnFrame is commented out - so retention has to be explicit rather than a
			// consequence of the scheduler stopping.
			if (Device.Paused())
				return;

			sink::instance().begin_frame();

			if (!g_explain || !g_pGameLevel)
				return;

			CObject* target = Actor();
			if (!target)
				return;

			const bool log_now = g_log && Device.dwTimeGlobal >= s_next_log;

			for (u32 i = 0; i < Level().Objects.o_count(); ++i)
			{
				CObject* o = Level().Objects.o_get_by_iterator(i);
				if (o == target)
					continue;

				CCustomMonster* m = smart_cast<CCustomMonster*>(o);
				if (!m)
					continue;
				if (g_explain_id != 0xffff && m->ID() != g_explain_id)
					continue;

				collect(m, target, false);
			}

			if (log_now)
				s_next_log = Device.dwTimeGlobal + u32(_max(g_log_interval, 50));
		}
	}
}

#endif
