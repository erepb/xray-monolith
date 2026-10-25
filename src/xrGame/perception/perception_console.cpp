#include "stdafx.h"
#include "perception_debug.h"

#ifdef PERCEPTION_DEBUG

#include "occlusion_query.h"
#include "shadow_harness.h"
#include "atmosphere.h"
#include "canopy_oracle.h"
#include "../Level.h"
#include "../../xrEngine/IGame_Persistent.h"   // pulls Environment.h
#include "../HUDManager.h"
#include "../../xrEngine/xr_ioconsole.h"
#include "../../xrEngine/xr_ioc_cmd.h"

namespace perception
{
	namespace debug
	{
		// The foliage channel is the one number with no engine counterpart to check it against, so the
		// probe reports the march's inputs beside its output: whether the grass field holds a slab
		// under each end of the ray, and where that slab sits relative to the ray's own height.
		// "no grass here" and "grass is there but the ray passed above it" both read as 1.000 in the
		// channel and call for completely different responses.
		static bool span_less(const Fvector2& a, const Fvector2& b) { return a.x < b.x; }

		static void report_grass(const segment& seg, const occlusion_result& r)
		{
			const foliage_oracle& fo = foliage_oracle::instance();
			if (!fo.ready())
			{
				Msg("~   grass  oracle not ready");
				return;
			}
			if (!fo.renderer_places_detail())
			{
				Msg("~   grass  this renderer places no detail objects - channel inactive");
				return;
			}

			float lo = 0.f, hi = 0.f;
			const bool ha = fo.band_at(seg.from.x, seg.from.z, lo, hi);
			Msg("~   grass  eye y=%.2f  band %s %.2f..%.2f", seg.from.y,
			    ha ? "yes " : "none", ha ? lo : 0.f, ha ? hi : 0.f);

			const bool hb = fo.band_at(seg.to.x, seg.to.z, lo, hi);
			Msg("~   grass  tgt y=%.2f  band %s %.2f..%.2f", seg.to.y,
			    hb ? "yes " : "none", hb ? lo : 0.f, hb ? hi : 0.f);

			// Crossings and the metres spent inside them, from the chords the march just recorded.
			// tau is their product with opacity, so all three together say whether a weak result came
			// from missing the grass or from grass that is genuinely thin.
			// Tufts overlap heavily - a 30 m ray crosses ~100 of them - so summing chord lengths
			// counts the same metre many times and reads as "89 m inside of 30 m". The meaningful
			// figure is the union, which is why the intervals are merged rather than added. Done here
			// and not in the march because it is a diagnostic and the runtime path should not pay
			// for it.
			const occlusion_query& q = occlusion_query::instance();

			xr_vector<Fvector2> spans;
			spans.reserve(q.last_chords.size());
			for (u32 i = 0; i < q.last_chords.size(); ++i)
			{
				Fvector2 sp;
				sp.set(seg.from.distance_to(q.last_chords[i].enter),
				       seg.from.distance_to(q.last_chords[i].exit));
				if (sp.y < sp.x)
					std::swap(sp.x, sp.y);
				spans.push_back(sp);
			}
			std::sort(spans.begin(), spans.end(), span_less);

			float covered = 0.f;
			for (u32 i = 0; i < spans.size();)
			{
				float lo = spans[i].x, hi = spans[i].y;
				u32 j = i + 1;
				for (; j < spans.size() && spans[j].x <= hi; ++j)
					hi = _max(hi, spans[j].y);
				covered += hi - lo;
				i = j;
			}

			Msg("~   grass  crossed %u tuft(s) over %.2f m of %.2f m", q.last_chords.size(), covered,
			    seg.length);

			if (r.foliage_unresolved)
				Msg("! [perception]   %u slot(s) unresolved this frame - counted as fully occluding, "
				    "will settle next frame", r.foliage_unresolved);

		}

		// Same split report_grass exists for: "no canopy here" and "canopy is there but the ray passed
		// under or over it" both read as 1.000 in the channel and call for different responses.
		static void report_canopy(const segment& seg, const occlusion_result& r)
		{
			const canopy_oracle& co = canopy_oracle::instance();
			if (!co.ready())
			{
				Msg("~   canopy no field on this level - channel inactive");
				if (r.fallback_hits)
					Msg("! [perception]   %u vegetation proxy hit(s) counted as soft - no canopy field "
					    "to defer to", r.fallback_hits);
				return;
			}

			float lo = 0.f, hi = 0.f;
			const bool ha = co.band_at(seg.from.x, seg.from.z, lo, hi);
			Msg("~   canopy eye y=%.2f  band %s %.2f..%.2f", seg.from.y, ha ? "yes " : "none",
			    ha ? lo : 0.f, ha ? hi : 0.f);

			const bool hb = co.band_at(seg.to.x, seg.to.z, lo, hi);
			Msg("~   canopy tgt y=%.2f  band %s %.2f..%.2f", seg.to.y, hb ? "yes " : "none",
			    hb ? lo : 0.f, hb ? hi : 0.f);

			// Cells are disjoint by construction, so unlike the grass tufts their lengths sum rather
			// than needing an interval union.
			const occlusion_query& q = occlusion_query::instance();
			float covered = 0.f;
			for (u32 i = 0; i < q.last_canopy.size(); ++i)
				covered += q.last_canopy[i].enter.distance_to(q.last_canopy[i].exit);

			Msg("~   canopy field %s", co.complete() ? "complete" : "INCOMPLETE - proxy hits also count");
			Msg("~   canopy crossed %u cell(s) over %.2f m of %.2f m   tau %.3f",
			    u32(r.canopy_cells), covered, seg.length, r.canopy_tau);

			if (r.deferred_hits)
				Msg("~   canopy %u vegetation proxy hit(s) ignored - the field owns that vegetation "
				    "(their transmittance %.3f)", r.deferred_hits, r.deferred_t);
			if (r.fallback_hits)
				Msg("! [perception]   %u vegetation proxy hit(s) counted as soft - canopy field incomplete",
				    r.fallback_hits);
		}

		static void report(const occlusion_result& r, const segment& seg, const occlusion_model& mdl)
		{
			const float dist = seg.length;
			Msg("~ [perception] probe  %.1f m", dist);
			Msg("~   transmittance %.3f   dominant %s (%s)", r.transmittance,
			    occluder_class_name(r.dominant),
			    r.dominant_material.size() ? r.dominant_material.c_str() : "-");
			// What a v1 trace would have produced, for the Stage 3 comparison.
			Msg("~   v1-comparable T %.3f", r.v1_transmittance());
			if (r.degraded)
			{
				string256 deg;
				Msg("! [perception]   DEGRADED: %s", describe_degraded(r.degraded, deg));
			}
			else
				Msg("~   measured on every channel");
			Msg("~   hard %.3f  soft %.3f  creature %.3f  foliage %.3f  canopy %.3f  atmosphere %.3f",
			    r.channel[occ_hard], r.channel[occ_soft], r.channel[occ_creature], r.channel[occ_foliage],
			    r.channel[occ_canopy], r.channel[occ_atmosphere]);
			if (r.foliage_unresolved)
				Msg("~   foliage tau  (unresolved - forced opaque)   hits %u   cache depth:%s geom:%s",
				    r.hit_count, r.from_cache_depth ? "hit" : "miss",
				    r.from_cache_geom ? "hit" : "miss");
			else
				Msg("~   foliage tau %.3f   hits %u   cache depth:%s geom:%s",
				    r.foliage_tau, r.hit_count,
				    r.from_cache_depth ? "hit" : "miss", r.from_cache_geom ? "hit" : "miss");
			report_grass(seg, r);
			report_canopy(seg, r);
			Msg("~   model  foliage_k=%.2f canopy_k=%.2f atmosphere_k=%.2f extend=%.2f max_slab=%.2f",
			    mdl.foliage_k, mdl.canopy_k, mdl.atmosphere_k, mdl.ray_extend, mdl.max_slab_h);

			// The atmospheric input alongside its output, so the report explains itself: x is where
			// this distance falls in the current fog envelope.
			const atmosphere& atmo = atmosphere::instance();
			const CEnvDescriptorMixer* env = g_pGamePersistent ? g_pGamePersistent->Environment().CurrentEnv : 0;
			Msg("~   fog    x=%.3f of [%.0f..%.0f] m   curve %s%s",
			    atmosphere::normalized_distance(dist),
			    env ? env->fog_near : 0.f, env ? env->fog_far : 0.f,
			    atmo.is_analytic() ? "analytic" : "measured",
			    atmo.ready() ? "" : " (pending)");
		}

		static void report_cache()
		{
			const static_occlusion_cache::stats& s = occlusion_query::instance().cache().get_stats();
			Msg("~ [perception] cache  probes %u  depth %u  geom %u  evictions %u",
			    s.probes, s.depth_hits, s.geom_hits, s.evictions);

			const cache_split& cs = g_cache_split;
			for (u32 i = 0; i < 2; ++i)
			{
				if (!cs.traces[i])
					continue;
				Msg("~   %-8s traces %6u  depth %5.1f %%  geom %5.1f %%  degraded %5.1f %%",
				    i ? "occluded" : "visible", cs.traces[i],
				    100.f * float(cs.depth_hits[i]) / float(cs.traces[i]),
				    100.f * float(cs.geom_hits[i]) / float(cs.traces[i]),
				    100.f * float(cs.degraded[i]) / float(cs.traces[i]));
			}

			// Grass resolve economics. slots_above is the number this design lives or dies on: it is
			// the share of crossed slots a ray flew over without paying anything, and on long shots
			// over open ground it should dominate.
			const foliage_oracle::stats& fs = foliage_oracle::instance().get_stats();
			if (fs.marches)
			{
				Msg("~   grass  %u march(es): %u slot(s) with foliage, %u above band (%.1f %%), %u entered",
				    fs.marches, fs.slots_walked, fs.slots_above,
				    fs.slots_walked ? (100.f * float(fs.slots_above) / float(fs.slots_walked)) : 0.f,
				    fs.slots_entered);
				const occlusion_model& mdl = occlusion_query::instance().tunables();
				Msg("~   grass  %u resolve(s) in %.1f ms = %.1f us each  |  cache %u/%u slot(s) %u instance(s)  |  %u budget denial(s) at %u/frame",
				    fs.resolve_requests, fs.resolve_us / 1000.f,
				    fs.resolve_requests ? (fs.resolve_us / float(fs.resolve_requests)) : 0.f,
				    fs.resolved_slots, mdl.resolve_cache_slots, fs.instances, fs.budget_denied,
				    mdl.resolve_budget);
			}

			// Canopy has no cache and no budget, so what matters is only how much of the walk the band
			// reject threw away before any cell was touched.
			const canopy_oracle::stats& cstats = canopy_oracle::instance().get_stats();
			if (cstats.cells)
				Msg("~   canopy field %u cell(s) from %u instance(s) / %u model(s), card area %.0f m2, bake %.1f ms, %s",
				    cstats.cells, cstats.instances, cstats.models, cstats.leaf_area, cstats.bake_ms,
					canopy_oracle::instance().complete() ? "complete" : "INCOMPLETE");
			if (cstats.marches)
				Msg("~   canopy %u march(es): %u column(s) with canopy, %u missed the band (%.1f %%), %u cell(s) entered",
				    cstats.marches, cstats.columns_walked, cstats.columns_skipped,
				    cstats.columns_walked ? (100.f * float(cstats.columns_skipped) / float(cstats.columns_walked)) : 0.f,
				    cstats.cells_entered);

			const occlusion_query& q = occlusion_query::instance();
			if (q.validate_n)
				Msg("~   geom cache error over %u hits: mean %.4f  max %.4f",
				    q.validate_n, q.validate_sum / float(q.validate_n), q.validate_max);
		}

		class CCC_PerceptionProbe : public IConsole_Command
		{
		public:
			CCC_PerceptionProbe(LPCSTR N) : IConsole_Command(N) { bEmptyArgsHandled = TRUE; }

			virtual void Execute(LPCSTR args)
			{
				occlusion_query& q = occlusion_query::instance();
				if (!g_pGameLevel || !q.ready())
				{
					Msg("! [perception] no level");
					return;
				}

				// The camera pick is already computed every frame (HUDManager.h:91), so the crosshair
				// needs no picking code of its own.
				SPickParam& pp = HUD().GetPick();
				const float range = (pp.result.range > EPS_L) ? pp.result.range : 100.f;

				Fvector to;
				to.mad(Device.vCameraPosition, Device.vCameraDirection, range);

				segment seg;
				if (!segment::make(Device.vCameraPosition, to, seg))
				{
					Msg("! [perception] degenerate probe segment");
					return;
				}

				// The flag is global: left on, every later trace keeps recording and forces the real
				// foliage march instead of the cached depth, so the probe must put it back.
				const bool was_recording = q.record;
				q.record = true;
				occlusion_result res;
				if (!q.trace(seg, CDB::OPT_CULL, 0, 0, res))
				{
					q.record = was_recording;
					Msg("! [perception] trace failed");
					return;
				}

				report(res, seg, q.model());
				report_cache();
				q.record = was_recording;

				sink& sk = sink::instance();
				sk.begin_sticky();

				ray_record rr;
				rr.from = seg.from;
				rr.to = seg.to;
				rr.transmittance = res.transmittance;
				rr.dominant = res.dominant;
				rr.freshness = 1.f;
				sk.add_sticky(rr);

				for (u32 i = 0; i < q.last_chords.size(); ++i)
					sk.add_sticky(q.last_chords[i]);
				for (u32 i = 0; i < q.last_canopy.size(); ++i)
					sk.add_sticky(q.last_canopy[i]);
				for (u32 i = 0; i < q.last_hits.size(); ++i)
					sk.add_sticky(q.last_hits[i]);
			}

			virtual void Info(TInfo& I) { xr_strcpy(I, "trace from the camera crosshair, draw and print"); }
		};

		class CCC_PerceptionExplain : public IConsole_Command
		{
		public:
			CCC_PerceptionExplain(LPCSTR N) : IConsole_Command(N) { bEmptyArgsHandled = TRUE; }

			virtual void Execute(LPCSTR args)
			{
				if (args && xr_strlen(args))
				{
					if (0 == xr_strcmp(args, "off"))
					{
						g_explain = 0;
						sink::instance().clear();
						Msg("~ [perception] explain off");
						return;
					}
					if (0 == xr_strcmp(args, "look"))
					{
						CObject* o = HUD().GetPick().result.O;
						if (!o)
						{
							Msg("! [perception] nothing under the crosshair");
							return;
						}
						g_explain_id = o->ID();
						g_explain = 1;
						Msg("~ [perception] explain %s [%u]", o->cName().c_str(), g_explain_id);
						return;
					}
					g_explain_id = u16(atoi(args));
					g_explain = 1;
					Msg("~ [perception] explain id %u", g_explain_id);
					return;
				}

				g_explain = !g_explain;
				g_explain_id = 0xffff;
				g_cache_split.reset();
				occlusion_query::instance().cache().reset_stats();
				foliage_oracle::instance().reset_stats();
				canopy_oracle::instance().reset_stats();
				occlusion_query::instance().reset_validation();
				if (!g_explain)
					sink::instance().clear();
				Msg("~ [perception] explain %s (all trackers of the actor)", g_explain ? "on" : "off");
				Msg("~   these are the primitive's numbers, not the NPC's belief - detection still runs on v1");
			}

			virtual void Info(TInfo& I)
			{
				xr_strcpy(I, "no arg: toggle all trackers | look: NPC under crosshair | <id> | off");
			}
		};

		class CCC_PerceptionCacheStats : public IConsole_Command
		{
		public:
			CCC_PerceptionCacheStats(LPCSTR N) : IConsole_Command(N) { bEmptyArgsHandled = TRUE; }

			virtual void Execute(LPCSTR args)
			{
				if (args && 0 == xr_strcmp(args, "reset"))
				{
					g_cache_split.reset();
					occlusion_query::instance().cache().reset_stats();
				foliage_oracle::instance().reset_stats();
				canopy_oracle::instance().reset_stats();
					occlusion_query::instance().reset_validation();
					Msg("~ [perception] cache stats reset");
					return;
				}
				report_cache();
			}

			virtual void Info(TInfo& I) { xr_strcpy(I, "print cache hit rates | reset"); }
		};

		class CCC_PerceptionShadow : public IConsole_Command
		{
		public:
			CCC_PerceptionShadow(LPCSTR N) : IConsole_Command(N) { bEmptyArgsHandled = TRUE; }

			virtual void Execute(LPCSTR args)
			{
				if (args && xr_strlen(args))
				{
					if (0 == xr_strcmp(args, "report"))
					{
						shadow::report();
						return;
					}
					if (0 == xr_strcmp(args, "reset"))
					{
						shadow::reset();
						Msg("~ [perception] shadow stats reset");
						return;
					}
					if (0 == xr_strcmp(args, "off")) shadow::g_mode = shadow::mode_off;
					else if (0 == xr_strcmp(args, "actor")) shadow::g_mode = shadow::mode_actor;
					else if (0 == xr_strcmp(args, "all")) shadow::g_mode = shadow::mode_all;
					else
					{
						Msg("! [perception] perception_shadow off|actor|all|report|reset");
						return;
					}
				}
				Msg("~ [perception] shadow %s", shadow::mode_name(shadow::g_mode));
			}

			virtual void Info(TInfo& I) { xr_strcpy(I, "off | actor | all | report | reset"); }
		};

		void register_console_commands()
		{
			CMD1(CCC_PerceptionProbe, "perception_probe");
			CMD1(CCC_PerceptionExplain, "perception_explain");
			CMD1(CCC_PerceptionCacheStats, "perception_cache");
			CMD1(CCC_PerceptionShadow, "perception_shadow");

			// Only the trims and ray_extend are live. max_slab_h and the resolve limits are copied into
			// the grass oracle at level load, so a console-bound value would never reach it.
			occlusion_model& mdl = occlusion_query::instance().tunables();
			CMD4(CCC_Float, "perception_ray_extend", &mdl.ray_extend, 0.f, 5.f);
			CMD4(CCC_Float, "perception_foliage_k", &mdl.foliage_k, 0.f, 100.f);
			CMD4(CCC_Float, "perception_canopy_k", &mdl.canopy_k, 0.f, 100.f);
			CMD4(CCC_Float, "perception_atmosphere_k", &mdl.atmosphere_k, 0.f, 100.f);

			CMD4(CCC_Integer, "perception_dbg_draw", &g_draw, 0, 1);
			CMD4(CCC_Integer, "perception_dbg_tufts", &g_draw_tufts, 0, 1);
			CMD4(CCC_Integer, "perception_dbg_log", &g_log, 0, 1);
			CMD4(CCC_Integer, "perception_dbg_log_interval", &g_log_interval, 50, 60000);
		}
	}
}

#endif
