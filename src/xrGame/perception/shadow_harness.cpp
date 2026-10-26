#include "stdafx.h"
#include "shadow_harness.h"

#ifdef PERCEPTION_DEBUG

#include "vision_sampler.h"
#include "occlusion_query.h"
#include "foliage_oracle.h"
#include "perception_debug.h"
#include "../Level.h"
#include "../Actor.h"
#include "../CustomMonster.h"
#include "../entity_alive.h"
#include "../visual_memory_manager.h"
#include "../../xrEngine/GameMtlLib.h"
#include "../../xrEngine/cl_intersect.h"
#include "../../xrEngine/xr_collide_form.h"
#include "../../Include/xrRender/Kinematics.h"

namespace perception
{
	namespace shadow
	{
		int g_mode = mode_off;

		static LPCSTR CFG_FILE = "creatures\\perception.ltx";
		static LPCSTR CFG_SECTION = "vision_sampler";

		// v1's nominal overshoot (Feel_Vision.cpp:212). It reaches past the point only on object
		// shapes, so it is used only to classify parity mismatches.
		static const float V1_EXTEND = 0.20f;
		static const float NOISE_EPS = 0.005f; // float noise at the point itself
		static const u32 OBSERVER_STALE_MS = 10000;
		static const u32 DETAIL_LOGS_PER_CLASS = 5;
		static const u32 EXPLAIN_FRESH_MS = 3000;    // a record fades out over this age
		static const u32 EXPLAIN_OBSERVER_MS = 2000; // an observer silent for longer is not drawn
		static const u32 POINTS_DEFAULT = 2;
		static const float CURRENT_EPS_DEFAULT = 0.10f;
		static const u32 REFRESH_VISITS_DEFAULT = 4;
		static const float VISIBLE_MIN_DEFAULT = 0.05f;
		static LPCSTR FEED_SECTION = "vision_feed";

		static u32 s_points = POINTS_DEFAULT;
		static float s_visible_min = VISIBLE_MIN_DEFAULT;

		LPCSTR mode_name(int mode)
		{
			switch (mode)
			{
			case mode_all: return "all";
			case mode_actor: return "actor";
			default: return "off";
			}
		}

		// Ordered as classify() tests them.
		enum mismatch_class
		{
			mm_self_edge = 0, // v1 hidden, target's own body hit right at the point: v1's verdict there is float noise
			mm_self_beyond,  // v1 hidden, target's own body hit past the point: v1 reached it, as box bones do
			mm_reach_static, // v1 hidden, level geometry just past the point, within v1's nominal overshoot
			mm_reach_object, // v1 hidden, an object just past the point, likewise
			mm_undrawn,      // v1 hidden by an AI-visible object nothing draws, such as an anomaly zone
			mm_v1_missed,    // v1 visible, a blocker v2 hits is absent from v1's spatial query on v1's short direction
			mm_v1_shape,     // v1 visible, v1's spatial query finds the blocker but its collision model misses v1's ray
			mm_v1_cache,     // v1's cached blocker triangle, as it was before this tick, crosses this ray
			mm_unexplained,
			mm_class_count
		};

		static LPCSTR mismatch_name(u32 c)
		{
			switch (c)
			{
			case mm_self_edge: return "self-edge";
			case mm_self_beyond: return "self-beyond";
			case mm_reach_static: return "reach-static";
			case mm_reach_object: return "reach-object";
			case mm_undrawn: return "undrawn";
			case mm_v1_cache: return "v1-cache";
			case mm_v1_missed: return "v1-missed";
			case mm_v1_shape: return "v1-shape";
			default: return "unexplained";
			}
		}

		static LPCSTR limiting_name(u32 c)
		{
			return c < occ_class_count ? occluder_class_name(occluder_class(c)) : "none";
		}

		enum { COV_BINS = 5 };

		static u32 coverage_bin(float c)
		{
			if (c <= 0.f) return 0;
			if (c <= .25f) return 1;
			if (c <= .5f) return 2;
			if (c <= .75f) return 3;
			return 4;
		}

		struct stats
		{
			// parity
			u32 parity_n, parity_agree, parity_self_decided;
			u32 parity_body_hidden, parity_sphere_hidden; // agreed hidden with an AI-visible object in the way
			u32 mismatch[mm_class_count][2]; // [class][v1 said visible]
			u32 logged[mm_class_count];

			// bias
			u32 bias_n, bias_v1_visible;
			double sum_cov_v1, sum_cov_v1_self, sum_self_share;

			// divergence: [v1 said visible][coverage bin]
			u32 div[2][COV_BINS];
			u32 div_limiting[2][COV_BINS][occ_class_count + 1]; // last = none
			u32 div_degraded[2][COV_BINS];

			u32 max_spread;
			u32 incomplete; // pair-ticks with no full rotation yet, or no usable record
			u32 unsampled;  // pair-ticks whose target has no shape the sampler can use

			// cost
			double v1_us, sampler_us, parity_us;
			u32 v1_ticks, v1_rays, sampler_rays, parity_rays;
			u32 sampler_refreshed, sampler_skipped; // dynamic-only refreshes; full traces with no concealment march
			u32 sampler_tri;                        // answered by the pair's blocking triangle, no ray
			sample_counts counts;

			// The feed's rule against v1's, both at v1's own point: [v1's rules visible][feed seen].
			u32 rule[2][2];
			u32 rule_limiting[occ_class_count + 1]; // v1's rules visible, feed unseen: the lowest channel
			u32 rule_hull;                          // v1's rules hidden, feed seen, across a hull the canopy replaces

			// FEED-DESIGN 2.1: v1's verdict against the sticky point's seen, [v1 visible][seen], and what
			// hid the sticky point where v1 sees the target.
			u32 seen[2][2];
			u32 seen_limiting[occ_class_count + 1];

			u32 pair_ticks;

			u32 t_start; // Device.dwTimeGlobal at reset; rates are per second since then

			void reset()
			{
				ZeroMemory(this, sizeof(*this));
				t_start = Device.dwTimeGlobal;
			}
		};

		static stats s_stats;

		struct observer_entry
		{
			u16 id;
			const CObject* object;
			u32 last_tick;
			vision_sampler sampler;

			// Explain view, refreshed each vision tick for the actor pair.
			xr_vector<debug::chord_record> chords;
			xr_vector<debug::point_record> hits;
			debug::v1_point_record v1_point;
			bool has_v1_point;
			u32 next_ray_log;

			observer_entry(const CCustomMonster& m, u32 now)
				: id(m.ID()), object(&m), last_tick(now), sampler(s32(m.ID())), has_v1_point(false), next_ray_log(0)
			{
			}
		};

		struct cache_before
		{
			const CObject* O;
			Fvector verts[3];
		};

		static xr_vector<observer_entry> s_observers;
		static xr_vector<u16> s_ids;
		static xr_vector<cache_before> s_cache_before;
		static collide::rq_results s_raw;
		static xr_vector<body_hit> s_bodies;
		static xr_vector<ISpatial*> s_v1_spatial;
		static xr_vector<const CObject*> s_v1_missed;
		static collide::rq_results s_v1_rq;
		static u32 s_next_log = 0;

		static bool active() { return g_mode != mode_off || debug::g_explain; }

		static bool explain_id_matches(u16 id)
		{
			return debug::g_explain_id == 0xffff || debug::g_explain_id == id;
		}

		static observer_entry& acquire(const CCustomMonster& m, u32 now)
		{
			for (u32 i = 0; i < s_observers.size(); ++i)
			{
				observer_entry& e = s_observers[i];
				if (e.id != m.ID())
					continue;
				if (e.object != &m)
				{
					e.object = &m;
					e.sampler.clear();
					e.has_v1_point = false;
				}
				e.last_tick = now;
				return e;
			}

			s_observers.push_back(observer_entry(m, now));
			return s_observers.back();
		}

		static void prune(u32 now)
		{
			for (u32 i = 0; i < s_observers.size();)
			{
				if (now - s_observers[i].last_tick > OBSERVER_STALE_MS)
				{
					s_observers[i] = s_observers.back();
					s_observers.pop_back();
				}
				else
					++i;
			}
		}

		static bool explained(const CCustomMonster& m, const CObject* target)
		{
			return debug::g_explain && target == Actor() && explain_id_matches(m.ID());
		}

		static bool v1_spatial_has(const CObject* O)
		{
			for (u32 i = 0; i < s_v1_spatial.size(); ++i)
				if (s_v1_spatial[i]->dcast_CObject() == O)
					return true;
			return false;
		}

		static const cache_before* find_cache_before(const CObject* O)
		{
			for (u32 i = 0; i < s_cache_before.size(); ++i)
				if (s_cache_before[i].O == O)
					return &s_cache_before[i];
			return 0;
		}

		// v1's ray as Feel_Vision.cpp:204-215 builds it: D is divided by the extended length, so it is
		// shorter than unit. q_ray's sphere test assumes a unit direction (_sphere.h:37-47).
		static void v1_ray(const segment& seg, const Fvector& point, Fvector& D, float& f)
		{
			D.sub(point, seg.from);
			f = D.magnitude() + V1_EXTEND;
			D.div(f);
		}

		// ---- parity ---------------------------------------------------------------------------

		// Both of v1's passes over the segment and v1's nominal overshoot past it.
		static void raw_hits(CCustomMonster& m, const CObject& target, const segment& seg)
		{
			const collide::rq_target tgt =
				collide::rq_target(collide::rqtStatic | collide::rqtObject | collide::rqtObstacle);
			const collide::ray_defs rd(seg.from, seg.dir, seg.length + V1_EXTEND, CDB::OPT_CULL, tgt);
			Level().ObjectSpace.RayQuery(s_raw, rd, 0, 0, vision_test_callback(), &m);
			ai_visible_hits(seg, seg.length + V1_EXTEND, &m, &target, false, s_bodies);
		}

		// Leaves s_raw and s_bodies holding the ray's hits.
		static mismatch_class classify(CCustomMonster& m, const CObject& target, const Feel::Vision::feel_visible_Item& item,
		                               const segment& seg, bool v1_visible)
		{
			raw_hits(m, target, seg);
			s_v1_missed.clear();

			// v2's ray ends at the point and v1's second pass only zeroes, so every class below explains
			// only v1 hidden / v2 visible.
			if (!v1_visible && s_raw.r_count())
			{
				for (collide::rq_result* h = s_raw.r_begin(); h != s_raw.r_end(); ++h)
				{
					if (h->O != &target)
						continue;
					if (h->range >= seg.length - SELF_EPS && h->range <= seg.length + NOISE_EPS)
						return mm_self_edge;
					if (h->range > seg.length + NOISE_EPS && h->range <= seg.length + V1_EXTEND)
						return mm_self_beyond;
					break; // sorted: the nearest own hit decides
				}

				for (collide::rq_result* h = s_raw.r_begin(); h != s_raw.r_end(); ++h)
				{
					if (h->range <= seg.length || h->O == &target)
						continue;
					if (material_transmittance(h->O, h->element) < 1.f)
						return h->O ? mm_reach_object : mm_reach_static;
				}
			}
			if (!v1_visible)
			{
				for (u32 i = 0; i < s_bodies.size(); ++i)
					if (!s_bodies[i].drawn && s_bodies[i].range <= seg.length)
						return mm_undrawn;
				for (u32 i = 0; i < s_bodies.size(); ++i)
					if (s_bodies[i].range > seg.length)
						return mm_reach_object;
			}

			Fvector D;
			float f;
			v1_ray(seg, item.cp_LAST, D, f);

			// Every blocker v2 hit on the segment, looked up in v1's own spatial queries: its second pass
			// (Feel_Vision.cpp:257) for AI-visible bodies, RayQuery's object pass (xr_area_raypick.cpp:330)
			// for obstacles.
			if (v1_visible)
			{
				g_SpatialSpace->q_ray(s_v1_spatial, 0, STYPE_VISIBLEFORAI, seg.from, D, f);
				for (u32 i = 0; i < s_bodies.size(); ++i)
				{
					const body_hit& b = s_bodies[i];
					if (b.O && b.drawn && b.range <= seg.length && !v1_spatial_has(b.O))
						s_v1_missed.push_back(b.O);
				}
				g_SpatialSpace->q_ray(s_v1_spatial, 0, STYPE_COLLIDEABLE | STYPE_OBSTACLE, seg.from, D, f);
				for (collide::rq_result* h = s_raw.r_begin(); s_raw.r_count() && h != s_raw.r_end(); ++h)
				{
					if (h->O && h->O != &target && h->range <= seg.length &&
					    material_transmittance(h->O, h->element) < 1.f && !v1_spatial_has(h->O))
						s_v1_missed.push_back(h->O);
				}
				if (!s_v1_missed.empty())
					return mm_v1_missed;

				// The same blockers, through v1's own collision-model calls: its second pass
				// (Feel_Vision.cpp:286-288) for bodies, RayQuery's object pass for obstacles.
				const collide::rq_target tgt =
					collide::rq_target(collide::rqtStatic | collide::rqtObject | collide::rqtObstacle);
				const collide::ray_defs second(seg.from, D, f, CDB::OPT_ONLYFIRST, tgt);
				const collide::ray_defs first(seg.from, D, f, CDB::OPT_CULL, tgt);
				for (u32 i = 0; i < s_bodies.size(); ++i)
				{
					const body_hit& b = s_bodies[i];
					if (!b.O || !b.drawn || b.range > seg.length || !b.O->collidable.model)
						continue;
					s_v1_rq.r_clear();
					if (!b.O->collidable.model->_RayQuery(second, s_v1_rq))
						s_v1_missed.push_back(b.O);
				}
				for (collide::rq_result* h = s_raw.r_begin(); s_raw.r_count() && h != s_raw.r_end(); ++h)
				{
					if (!h->O || h->O == &target || h->range > seg.length || !h->O->collidable.model ||
					    !(material_transmittance(h->O, h->element) < 1.f))
						continue;
					s_v1_rq.r_clear();
					if (!h->O->collidable.model->_RayQuery(first, s_v1_rq))
						s_v1_missed.push_back(h->O);
				}
				if (!s_v1_missed.empty())
					return mm_v1_shape;
			}

			// v1's blocker triangle as it was when this tick began.
			if (const cache_before* before = find_cache_before(item.O))
			{
				Fvector verts[3] = {before->verts[0], before->verts[1], before->verts[2]};
				float u, v, range;
				if (!verts[0].similar(verts[1], EPS_S) && CDB::TestRayTri(seg.from, D, verts, u, v, range, false) &&
					range > 0 && range < f)
					return mm_v1_cache;
			}

			return mm_unexplained;
		}

		static void log_mismatch(CCustomMonster& m, const CObject& target, mismatch_class c, bool v1_visible,
		                         const segment& seg, const occlusion_result& r, float thr)
		{
			Msg("! [perception] parity %-11s %s -> %s  v1 %s  v2-as-v1 %s  len %.2f  t_v1 %.3f  thr %.2f  self %u @ %.2f",
			    mismatch_name(c), m.cName().c_str(), target.cName().c_str(), v1_visible ? "visible" : "hidden",
			    v1_visible ? "hidden" : "visible", seg.length, r.v1_transmittance(), thr, u32(r.self_hits),
			    r.self_hits ? r.self_range : 0.f);

			if (mm_v1_missed == c || mm_v1_shape == c)
			{
				const float k = seg.length / (seg.length + V1_EXTEND);
				for (u32 i = 0; i < s_v1_missed.size(); ++i)
					Msg("~   v1's %s, direction length %.4f, misses %s",
					    mm_v1_missed == c ? "spatial query" : "collision-model test", k, s_v1_missed[i]->cName().c_str());
			}
			if (!s_raw.r_count() && s_bodies.empty())
			{
				Msg("~   no hits");
				return;
			}
			for (collide::rq_result* h = s_raw.r_begin(); s_raw.r_count() && h != s_raw.r_end(); ++h)
			{
				const SGameMtl* mtl = GMLib.GetMaterialByIdx(material_of(*h));
				Msg("~   hit %6.2f  %-28s %-28s vis %.2f%s", h->range, h->O ? h->O->cName().c_str() : "static",
				    mtl ? mtl->m_Name.c_str() : "?", material_transmittance(h->O, h->element),
				    h->O == &target ? "  (target)" : "");
			}
			for (u32 i = 0; i < s_bodies.size(); ++i)
			{
				const body_hit& b = s_bodies[i];
				Msg("~   body %6.2f  %-28s %s, %s", b.range, b.O ? b.O->cName().c_str() : "(not an object)",
				    b.sphere ? "bounding sphere, no collision model" : "collision model",
				    b.drawn ? "drawn" : "not drawn");
			}
		}

		static void parity(CCustomMonster& m, const Fvector& eye, const CEntityAlive& target,
		                   const Feel::Vision::feel_visible_Item& item, float thr, bool v1_visible)
		{
			segment seg;
			if (!segment::make(eye, item.cp_LAST, seg))
				return;

			CTimer t;
			t.Start();
			occlusion_result r;
			trace_options opt;
			opt.clear_threshold = thr;
			opt.visible_min = s_visible_min;
			const bool ok = occlusion_query::instance().trace(seg, CDB::OPT_CULL, &m, &target, r, opt);
			s_stats.parity_us += t.GetElapsed_sec() * 1000000.0;
			++s_stats.parity_rays;
			if (!ok)
				return;

			const bool self_blocked = r.self_occluded(seg.length);
			const bool predicted = !self_blocked && r.v1_transmittance() >= thr;

			const bool feed_seen = !self_blocked && !point_fails(point_visibility(r.channel, thr), s_visible_min);
			++s_stats.rule[predicted ? 1 : 0][feed_seen ? 1 : 0];
			if (predicted && !feed_seen)
				++s_stats.rule_limiting[lowest_channel(r.channel)];
			else if (!predicted && feed_seen && r.deferred_hits)
				++s_stats.rule_hull;

			++s_stats.parity_n;
			if (predicted == v1_visible)
			{
				++s_stats.parity_agree;
				if (self_blocked && r.v1_transmittance() >= thr)
					++s_stats.parity_self_decided;
				if (!v1_visible && r.body_hits)
				{
					++s_stats.parity_body_hidden;
					if (r.sphere_hits)
						++s_stats.parity_sphere_hidden;
				}
				return;
			}

			const mismatch_class c = classify(m, target, item, seg, v1_visible);
			++s_stats.mismatch[c][v1_visible ? 1 : 0];
			if (s_stats.logged[c] < DETAIL_LOGS_PER_CLASS)
			{
				++s_stats.logged[c];
				log_mismatch(m, target, c, v1_visible, seg, r, thr);
			}
		}

		// ---- sampler --------------------------------------------------------------------------

		static const u32 RAY_LOG_ITEMS = 32; // per list, per ray

		static float along(const segment& seg, const Fvector& p)
		{
			Fvector d;
			d.sub(p, seg.from);
			return d.dotproduct(seg.dir);
		}

		static void log_chords(LPCSTR label, const segment& seg, const xr_vector<veg_chord>& chords)
		{
			for (u32 i = 0; i < chords.size() && i < RAY_LOG_ITEMS; ++i)
			{
				const veg_chord& c = chords[i];
				if (u32(-1) != c.crown)
					Msg("~     %s %6.2f..%6.2f m  y %7.2f..%7.2f  crown %u model %u  hits %u  tau %.3f%s", label,
					    along(seg, c.enter), along(seg, c.exit), c.enter.y, c.exit.y, c.crown, u32(c.model),
					    u32(c.hits), c.tau, c.inside ? "  (eye inside)" : "");
				else if (c.radius > 0.f && c.y_hi > c.y_lo)
				{
					// Closest approach of the ray's horizontal line to the tuft's axis.
					const float dx = c.origin.x - seg.from.x, dz = c.origin.z - seg.from.z;
					const float m2 = _sqrt(seg.dir.x * seg.dir.x + seg.dir.z * seg.dir.z);
					const float off = m2 > EPS_S ? _abs(dx * seg.dir.z - dz * seg.dir.x) / m2 : _sqrt(dx * dx + dz * dz);
					const float h = c.y_hi - c.y_lo;
					Msg("~     %s %6.2f..%6.2f m  y %7.2f..%7.2f  tau %.3f  model %u x%.2f  R %.2f core %.2f  axis at %3.0f %% of R  ray at %3.0f..%3.0f %% of h (core top %3.0f %%)",
					    label, along(seg, c.enter), along(seg, c.exit), c.enter.y, c.exit.y, c.tau, u32(c.model),
					    c.scale, c.radius, c.core_radius, 100.f * off / c.radius, 100.f * (c.enter.y - c.y_lo) / h,
					    100.f * (c.exit.y - c.y_lo) / h, 100.f * (c.core_top - c.y_lo) / h);
				}
				else
					Msg("~     %s %6.2f..%6.2f m  y %7.2f..%7.2f  blocked %.3f", label, along(seg, c.enter),
					    along(seg, c.exit), c.enter.y, c.exit.y, c.blocked);
			}
			if (chords.size() > RAY_LOG_ITEMS)
				Msg("~     %s ... %u more", label, u32(chords.size() - RAY_LOG_ITEMS));
		}

		// Needs occlusion_query::record set for the trace, or the chord and hit lists are empty.
		static void log_ray(const CCustomMonster& m, const CObject& target, const segment& seg, u16 bone,
		                    const occlusion_result& r)
		{
			IKinematics* K = smart_cast<IKinematics*>(target.Visual());
			string256 deg;
			Msg("~ [perception] ray %s -> %s  %s  len %.2f  y %.2f -> %.2f  T %.3f  v1 %.3f  dom %s %s",
			    m.cName().c_str(), target.cName().c_str(), K ? K->LL_BoneName_dbg(bone) : "?", seg.length,
			    seg.from.y, seg.to.y, r.transmittance, r.v1_transmittance(), occluder_class_name(r.dominant),
			    r.dominant_material.size() ? r.dominant_material.c_str() : "-");
			Msg("~   hard %.3f  soft %.3f  creature %.3f  foliage %.3f  canopy %.3f  atmo %.3f  self %u @ %.2f  bodies %u  degraded %s",
			    r.channel[occ_hard], r.channel[occ_soft], r.channel[occ_creature], r.channel[occ_foliage],
			    r.channel[occ_canopy], r.channel[occ_atmosphere], u32(r.self_hits), r.self_hits ? r.self_range : 0.f,
			    u32(r.body_hits), debug::describe_degraded(r.degraded, deg));

			const occlusion_query& q = occlusion_query::instance();
			Msg("~   grass tau %.3f  %u tuft(s)  %u slot(s) unresolved  |  canopy tau %.3f  %u crown(s) hit",
			    r.foliage_tau, u32(q.last_chords.size()), u32(r.foliage_unresolved), r.canopy_tau,
			    u32(r.canopy_crowns));
			log_chords("tuft", seg, q.last_chords);
			log_chords("leaf", seg, q.last_canopy);
			for (u32 i = 0; i < q.last_hits.size() && i < RAY_LOG_ITEMS; ++i)
			{
				const hit_mark& h = q.last_hits[i];
				Msg("~     hit  %6.2f m  y %7.2f  %s  t %.3f", along(seg, h.pos), h.pos.y, occluder_class_name(h.cls),
				    h.t);
			}
			if (q.last_hits.size() > RAY_LOG_ITEMS)
				Msg("~     hit  ... %u more", u32(q.last_hits.size() - RAY_LOG_ITEMS));
		}

		class listener : public trace_listener
		{
		public:
			float threshold;
			observer_entry* draw;
			const CCustomMonster* log_observer; // null: rays are not logged
			const CObject* log_target;

			virtual void on_trace(const segment& seg, u16 bone, const occlusion_result& r)
			{
				++s_stats.sampler_rays;
				if (r.refreshed)
					++s_stats.sampler_refreshed;
				if (r.concealment_skipped)
					++s_stats.sampler_skipped;
				if (r.tri_blocked)
					++s_stats.sampler_tri;
				debug::g_cache_split.note(r.transmittance < threshold, r);
				if (log_observer)
					log_ray(*log_observer, *log_target, seg, bone, r);
				if (!draw)
					return;

				const occlusion_query& q = occlusion_query::instance();
				draw->chords.insert(draw->chords.end(), q.last_chords.begin(), q.last_chords.end());
				draw->chords.insert(draw->chords.end(), q.last_canopy.begin(), q.last_canopy.end());
				draw->hits.insert(draw->hits.end(), q.last_hits.begin(), q.last_hits.end());
			}
		};

		static void accumulate(const pair_summary& s, bool v1_visible)
		{
			if (s.sampled < s.slots)
			{
				++s_stats.incomplete;
				return;
			}

			++s_stats.bias_n;
			if (v1_visible)
				++s_stats.bias_v1_visible;
			s_stats.sum_cov_v1 += s.coverage_v1;
			s_stats.sum_cov_v1_self += s.coverage_v1_self;
			s_stats.sum_self_share += s.self_share;

			const u32 row = v1_visible ? 1 : 0;
			const u32 bin = coverage_bin(s.coverage);
			++s_stats.div[row][bin];
			++s_stats.div_limiting[row][bin][s.limiting]; // `none` lands in the last column
			if (s.degraded)
				++s_stats.div_degraded[row][bin];

			s_stats.max_spread = _max(s_stats.max_spread, s.visit_spread);
		}

		static void accumulate_seen(const pair_summary& s, bool v1_visible)
		{
			++s_stats.seen[v1_visible ? 1 : 0][s.seen ? 1 : 0];
			if (v1_visible && !s.seen)
				++s_stats.seen_limiting[s.sticky_dominant];
		}

		static void log_pair(const CCustomMonster& m, const CObject& target, bool v1_visible, const pair_summary& s)
		{
			string256 deg;
			Msg("~ [perception] shadow %-22s -> %-22s v1 %-7s seen %-3s best %.2f vis %.2f  cov %.2f  cov_v1 %.2f  T %.3f  lim %-10s age %u..%u ms  %u/%u slots (%u current)  degraded %s",
			    m.cName().c_str(), target.cName().c_str(), v1_visible ? "visible" : "hidden", s.seen ? "yes" : "no",
			    s.best, s.visibility, s.coverage, s.coverage_v1, s.transmittance, limiting_name(s.limiting),
			    s.newest_age, s.oldest_age, u32(s.sampled), u32(s.slots), u32(s.current),
			    debug::describe_degraded(s.degraded, deg));
		}

		// ---- tick -----------------------------------------------------------------------------

		tick::tick(CCustomMonster& observer, float dt) : m_observer(observer), m_dt(dt)
		{
			if (active())
			{
				// v1 overwrites each item's cached blocker during its own query.
				s_cache_before.clear_not_free();
				for (u32 i = 0; i < observer.feel_visible.size(); ++i)
				{
					const Feel::Vision::feel_visible_Item& item = observer.feel_visible[i];
					cache_before b;
					b.O = item.O;
					for (u32 v = 0; v < 3; ++v)
						b.verts[v] = item.Cache.verts[v];
					s_cache_before.push_back(b);
				}
			}

			m_timer.Start();
		}

		tick::~tick()
		{
			const double v1_us = m_timer.GetElapsed_sec() * 1000000.0;

			if (!active())
				return;
			// dt == 0 leaves v1's fuzzy unchanged, so this tick carries no verdict of its own.
			if (m_dt <= 0.f || !g_pGameLevel || !m_observer.g_Alive())
				return;

			occlusion_query& q = occlusion_query::instance();
			if (!q.ready())
				return;

			CCustomMonster& m = m_observer;
			const CVisualMemoryManager* vm = m.visual_memory();
			const float thr = vm ? vm->transparency_threshold() : 0.5f;
			const u32 now = Device.dwTimeGlobal;
			const Fvector eye = m.eye_matrix.c;

			observer_entry& e = acquire(m, now);
			s_ids.clear();

			const bool log_now = debug::g_log && now >= s_next_log;
			bool any = false;
			u32 traced_by_v1 = 0;

			for (u32 i = 0; i < m.feel_visible.size(); ++i)
			{
				const Feel::Vision::feel_visible_Item& item = m.feel_visible[i];
				if (item.O && item.O->CFORM())
					++traced_by_v1;

				CEntityAlive* target = smart_cast<CEntityAlive*>(item.O);
				if (!target || !target->CFORM())
					continue;

				const bool shown = explained(m, target);
				if (!shown && g_mode != mode_all && !(g_mode == mode_actor && target == Actor()))
					continue;

				s_ids.push_back(target->ID());
				if (item.cp_LAST.similar(eye, EPS_L))
					continue;

				any = true;
				// The same test feel_vision_get applies.
				const bool v1_visible = positive(item.fuzzy);

				parity(m, eye, *target, item, thr, v1_visible);

				listener l;
				l.threshold = thr;
				const bool draw = shown && debug::g_draw;
				const bool log_rays = shown && debug::g_log && now >= e.next_ray_log;
				l.draw = draw ? &e : 0;
				l.log_observer = log_rays ? &m : 0;
				l.log_target = target;
				if (log_rays)
					e.next_ray_log = now + u32(_max(debug::g_log_interval, 50));
				if (shown)
				{
					e.chords.clear();
					e.hits.clear();
					e.v1_point.pos = item.cp_LAST;
					e.v1_point.visible = v1_visible;
					e.has_v1_point = true;
				}

				const bool was_recording = q.record;
				q.record = draw || log_rays;
				CTimer t;
				t.Start();
				trace_options limits;
				limits.clear_threshold = thr;
				limits.visible_min = s_visible_min;
				e.sampler.sample(&m, eye, target, s_points, now, limits, &s_stats.counts, &l);
				s_stats.sampler_us += t.GetElapsed_sec() * 1000000.0;
				q.record = was_recording;

				++s_stats.pair_ticks;
				pair_summary s;
				if (!e.sampler.summary(target, eye, thr, s_visible_min, now, s))
				{
					if (!s.slots)
						++s_stats.unsampled;
					else
						++s_stats.incomplete;
					continue;
				}

				if (s.has_sticky)
					accumulate_seen(s, v1_visible);
				if (s_points > 1)
					accumulate(s, v1_visible);
				if (log_now)
					log_pair(m, *target, v1_visible, s);
			}

			e.sampler.retain_only(s_ids);

			if (any)
			{
				s_stats.v1_us += v1_us;
				++s_stats.v1_ticks;
				s_stats.v1_rays += traced_by_v1;
			}
			if (log_now && any)
				s_next_log = now + u32(_max(debug::g_log_interval, 50));

			prune(now);
		}

		// ---- explain view ---------------------------------------------------------------------

		void draw_explained()
		{
			CObject* actor = Actor();
			if (!actor)
				return;

			debug::sink& sk = debug::sink::instance();
			const u32 now = Device.dwTimeGlobal;

			for (u32 i = 0; i < s_observers.size(); ++i)
			{
				const observer_entry& e = s_observers[i];
				if (!explain_id_matches(e.id) || now - e.last_tick > EXPLAIN_OBSERVER_MS)
					continue;

				const xr_vector<slot_record>* recs = e.sampler.records(actor->ID());
				if (!recs)
					continue;

				for (u32 r = 0; r < recs->size(); ++r)
				{
					const slot_record& rec = (*recs)[r];
					if (!rec.time)
						continue;

					debug::ray_record rr;
					rr.from = rec.eye;
					rr.to = rec.point;
					rr.transmittance = rec.t;
					rr.dominant = rec.dominant;
					rr.freshness = 1.f - _min(float(now - rec.time) / float(EXPLAIN_FRESH_MS), 1.f);
					sk.add(rr);
				}

				if (const slot_record* st = e.sampler.sticky(actor->ID()))
				{
					if (st->time)
					{
						debug::ray_record rr;
						rr.from = st->eye;
						rr.to = st->point;
						rr.transmittance = st->t;
						rr.dominant = st->dominant;
						rr.freshness = 1.f - _min(float(now - st->time) / float(EXPLAIN_FRESH_MS), 1.f);
						rr.sticky = true;
						sk.add(rr);
					}
				}

				for (u32 c = 0; c < e.chords.size(); ++c)
					sk.add(e.chords[c]);
				for (u32 h = 0; h < e.hits.size(); ++h)
					sk.add(e.hits[h]);
				if (e.has_v1_point)
					sk.add(e.v1_point);
			}
		}

		// ---- lifecycle and report -------------------------------------------------------------

		void on_level_load()
		{
			s_observers.clear();
			s_points = POINTS_DEFAULT;
			float current_eps = CURRENT_EPS_DEFAULT;
			u32 refresh_visits = REFRESH_VISITS_DEFAULT;
			s_visible_min = VISIBLE_MIN_DEFAULT;
			if (!s_stats.t_start)
				s_stats.t_start = Device.dwTimeGlobal;

			string_path fn;
			if (!FS.exist(fn, "$game_config$", CFG_FILE))
				Msg("~ [perception] %s absent, vision sampler at defaults", CFG_FILE);
			else
			{
				CInifile* ltx = xr_new<CInifile>(fn, TRUE);
				if (ltx->line_exist(FEED_SECTION, "visible_min"))
					s_visible_min = ltx->r_float(FEED_SECTION, "visible_min");
				if (!ltx->section_exist(CFG_SECTION))
					Msg("~ [perception] [%s] absent from %s, vision sampler at defaults", CFG_SECTION, CFG_FILE);
				else
				{
					if (ltx->line_exist(CFG_SECTION, "points"))
						s_points = ltx->r_u32(CFG_SECTION, "points");
					if (ltx->line_exist(CFG_SECTION, "current_eps"))
						current_eps = ltx->r_float(CFG_SECTION, "current_eps");
					if (ltx->line_exist(CFG_SECTION, "refresh_visits"))
						refresh_visits = ltx->r_u32(CFG_SECTION, "refresh_visits");
				}
				xr_delete(ltx);
			}
			clamp(s_points, 1u, 64u);
			clamp(current_eps, 0.f, 1.f);
			clamp(refresh_visits, 1u, 64u);
			clamp(s_visible_min, 0.f, 1.f);
			vision_sampler::configure(current_eps, refresh_visits);
			Msg("~ [perception] vision sampler: %u point(s) per vision tick, refresh within %.2f m, full every %u visit(s), visible_min %.2f",
			    s_points, current_eps, refresh_visits, s_visible_min);
		}

		void on_level_unload()
		{
			s_observers.clear();
			s_cache_before.clear_not_free();
			s_raw.r_clear();
		}

		void reset()
		{
			s_stats.reset();
			s_observers.clear();
			debug::g_cache_split.reset();
		}

		static float pct(double a, double b) { return b > 0. ? float(100. * a / b) : 0.f; }

		void report()
		{
			const stats& s = s_stats;
			Msg("~ [perception] shadow report  mode %s  points %u", mode_name(g_mode), s_points);

			Msg("~ parity      %u check(s), %.1f %% agree, %u decided by the target's own body",
			    s.parity_n, pct(s.parity_agree, s.parity_n), s.parity_self_decided);
			Msg("~             agreed hidden by an AI-visible object where the materials did not block: %u (%u counted on the bounding sphere)",
			    s.parity_body_hidden, s.parity_sphere_hidden);
			for (u32 c = 0; c < mm_class_count; ++c)
			{
				if (s.mismatch[c][0] || s.mismatch[c][1])
					Msg("~   %-11s v1 visible, v2 hidden %5u   v1 hidden, v2 visible %5u", mismatch_name(c),
					    s.mismatch[c][1], s.mismatch[c][0]);
			}

			const u32 rule_n = s.rule[0][0] + s.rule[0][1] + s.rule[1][0] + s.rule[1][1];
			if (rule_n)
			{
				Msg("~ rule        %u check(s) at v1's own point: v1's rules and the feed agree %.1f %%", rule_n,
				    pct(s.rule[0][0] + s.rule[1][1], rule_n));
				string512 lim = "";
				for (u32 c = 0; c <= occ_class_count; ++c)
				{
					if (!s.rule_limiting[c])
						continue;
					string64 one;
					xr_sprintf(one, " %s %.0f%%", limiting_name(c), pct(s.rule_limiting[c], s.rule[1][0]));
					xr_strcat(lim, one);
				}
				Msg("~   v1 rules visible, feed unseen %6u  lowest channel:%s", s.rule[1][0], lim);
				Msg("~   v1 rules hidden,  feed seen   %6u  of them %u across a tree hull the canopy replaces",
				    s.rule[0][1], s.rule_hull);
			}

			const bool rotating = s_points > 1;
			if (rotating && s.bias_n)
			{
				Msg("~ bias        %u pair-tick(s): v1 visible %.1f %%  |  v2 on v1 rules %.1f %%  (self-occlusion counted)  |  %.1f %% (not counted)",
				    s.bias_n, pct(s.bias_v1_visible, s.bias_n), pct(s.sum_cov_v1_self, s.bias_n),
				    pct(s.sum_cov_v1, s.bias_n));
				Msg("~             self-occluded under v1 rules: %.1f %% of area", pct(s.sum_self_share, s.bias_n));
			}
			if (rotating && s.incomplete)
				Msg("~             %u pair-tick(s) skipped: no full rotation yet, or no usable record", s.incomplete);
			if (s.unsampled)
				Msg("~ unsampled   %u pair-tick(s) on a target with no shape the sampler can use", s.unsampled);

			const u32 seen_n = s.seen[0][0] + s.seen[0][1] + s.seen[1][0] + s.seen[1][1];
			if (seen_n)
			{
				Msg("~ seen        %u pair-tick(s), v1's point against the sticky point: agree %.1f %%", seen_n,
				    pct(s.seen[0][0] + s.seen[1][1], seen_n));
				string512 lim = "";
				for (u32 c = 0; c <= occ_class_count; ++c)
				{
					if (!s.seen_limiting[c])
						continue;
					string64 one;
					xr_sprintf(one, " %s %.0f%%", limiting_name(c), pct(s.seen_limiting[c], s.seen[1][0]));
					xr_strcat(lim, one);
				}
				Msg("~   v1 visible, sticky unseen %6u  lowest channel:%s", s.seen[1][0], lim);
				Msg("~   v1 hidden,  sticky seen   %6u", s.seen[0][1]);
			}

			static const LPCSTR bins[COV_BINS] = {"0", "<=.25", "<=.50", "<=.75", ">.75"};
			for (u32 row = 0; rotating && row < 2; ++row)
			{
				Msg("~ divergence  v1 %s, by v2 coverage:", row ? "visible" : "hidden");
				for (u32 b = 0; b < COV_BINS; ++b)
				{
					const u32 n = s.div[row][b];
					if (!n)
						continue;

					string512 lim;
					lim[0] = 0;
					for (u32 c = 0; c <= occ_class_count; ++c)
					{
						if (!s.div_limiting[row][b][c])
							continue;
						string64 one;
						xr_sprintf(one, " %s %.0f%%", limiting_name(c),
						           pct(s.div_limiting[row][b][c], n));
						xr_strcat(lim, one);
					}
					Msg("~   %-6s %6u  degraded %5.1f %%  limiting:%s", bins[b], n, pct(s.div_degraded[row][b], n), lim);
				}
			}

			if (rotating)
				Msg("~ uniformity  max visit spread %u (rotation is exact at 1)", s.max_spread);

			if (s.v1_ticks)
			{
				Msg("~ cost/ray    v1 %.1f us  |  sampler %.1f us  |  parity %.1f us  |  %.2f sampler ray(s) per v1 ray",
				    s.v1_rays ? float(s.v1_us / s.v1_rays) : 0.f,
				    s.sampler_rays ? float(s.sampler_us / s.sampler_rays) : 0.f,
				    s.parity_rays ? float(s.parity_us / s.parity_rays) : 0.f,
				    s.v1_rays ? float(s.sampler_rays) / float(s.v1_rays) : 0.f);
				Msg("~ cost/tick   v1 %.1f us (its whole tick, every target)  |  sampler %.1f us (only the sampled pairs)  |  parity %.1f us",
				    float(s.v1_us / s.v1_ticks), float(s.sampler_us / s.v1_ticks), float(s.parity_us / s.v1_ticks));
				Msg("~ sampler     %u trace(s): %.1f %% refreshed against dynamic objects only, %.1f %% hidden by a blocking triangle with no ray, %.1f %% not marched behind materials or fog",
				    s.sampler_rays, pct(s.sampler_refreshed, s.sampler_rays), pct(s.sampler_tri, s.sampler_rays),
				    pct(s.sampler_skipped, s.sampler_rays));
				const sample_counts& c = s.counts;
				u32 full = 0;
				for (u32 i = 0; i < full_reason_count; ++i)
					full += c.full[i];
				Msg("~             %u full trace(s): new point %.1f %%, not reusable %.1f %%, eye moved %.1f %%, point moved %.1f %%, visit limit %.1f %%",
				    full, pct(c.full[full_new_point], full), pct(c.full[full_not_reusable], full),
				    pct(c.full[full_eye_moved], full), pct(c.full[full_point_moved], full),
				    pct(c.full[full_visit_limit], full));
				Msg("~             %u visit(s) with no ray: still, behind static geometry  |  sticky rerolls %u to a visible rotating record, %u drawn",
				    c.skipped_static, c.rerolls_best, c.rerolls_fresh);
			}

			const float secs = float(Device.dwTimeGlobal - s.t_start) / 1000.f;
			if (s.t_start && secs > 0.f)
				Msg("~ rates       over %.0f s:  sampler %.1f rays/s  |  parity %.1f rays/s  |  %.1f pair-ticks/s",
				    secs, s.sampler_rays / secs, s.parity_rays / secs,
				    s.pair_ticks / secs);

			const foliage_oracle::stats& fs = foliage_oracle::instance().get_stats();
			Msg("~             grass budget denials %u (since the last perception_cache reset)", fs.budget_denied);
		}
	}
}

#endif
