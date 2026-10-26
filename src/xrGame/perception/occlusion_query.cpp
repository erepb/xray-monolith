#include "stdafx.h"
#include "occlusion_query.h"
#include "atmosphere.h"
#include "canopy_oracle.h"
#include "perception_debug.h"
#include "shadow_harness.h"
#include "vision_feed.h"
#include "../Level.h"
#include "../entity_alive.h"
#include "../../Include/xrRender/Kinematics.h"
#include "../../xrEngine/GameMtlLib.h"
#include "../../xrEngine/xr_collide_form.h"
#include "../../xrEngine/cl_intersect.h"

namespace perception
{
#ifdef PERCEPTION_DEBUG
	// One in this many blocking-triangle answers is traced in full anyway, to prove the shortcut.
	static const u32 TRI_CHECK_EVERY = 16;
#endif
	float material_transmittance(CObject* O, u32 element)
	{
		float vis = 1.f;
		if (O)
		{
			IKinematics* V = smart_cast<IKinematics*>(O->Visual());
			if (0 != V)
			{
				CBoneData& B = V->LL_GetData((u16)element);
				vis = GMLib.GetMaterialByIdx(B.game_mtl_idx)->fVisTransparencyFactor;
			}
		}
		else
		{
			CDB::TRI* T = Level().ObjectSpace.GetStaticTris() + element;
			vis = GMLib.GetMaterialByIdx(T->material)->fVisTransparencyFactor;
		}
		return vis;
	}

	u16 material_of(const collide::rq_result& h)
	{
		if (h.O)
		{
			IKinematics* V = smart_cast<IKinematics*>(h.O->Visual());
			return V ? V->LL_GetData((u16)h.element).game_mtl_idx : u16(0);
		}
		return u16((Level().ObjectSpace.GetStaticTris() + h.element)->material);
	}

	static occluder_class classify(const collide::rq_result& h, u16 mtl, const occluder_table& table)
	{
		// By object, not by material: more robust than hoping every creature bone carries a creature
		// material, and the pointer is already in hand.
		if (h.O && smart_cast<CEntityAlive*>(h.O))
			return occ_creature;
		return table.of(mtl);
	}

	occlusion_query& occlusion_query::instance()
	{
		static occlusion_query s_query;
		return s_query;
	}

	void occlusion_query::on_level_load()
	{
		PDBG_SCOPE("occlusion_query::on_level_load");

		m_model.load();
		m_table.on_level_load();
		m_cache.clear();
		m_cache.reset_stats();

		foliage_oracle::instance().set_max_slab_h(m_model.max_slab_h);
		foliage_oracle::instance().set_resolve_limits(m_model.resolve_budget, m_model.resolve_cache_slots);
		foliage_oracle::instance().set_alpha_ref(m_model.alpha_ref);
		canopy_oracle::instance().set_bake_params(m_model.canopy_cone_eye, m_model.alpha_ref);
#ifdef PERCEPTION_DEBUG
		march_cost.reset();
		ray_cost.reset();
#endif
		m_ready = true;
	}

	void occlusion_query::on_level_unload()
	{
		m_table.on_level_unload();
		m_cache.clear();
		m_hits.r_clear();
#ifdef PERCEPTION_DEBUG
		last_chords.clear();
		last_canopy.clear();
		last_canopy_hits.clear();
		last_hits.clear();
		record = false;
		reset_validation();
#endif
		m_ready = false;
	}

#ifdef SPATIAL_CHANGE
	// Same rule as v1's test callback (Feel_Vision.cpp:53-61): objects flagged for vision to ignore,
	// such as a mounted gun in use, are not tested at all.
	static BOOL feel_vision_ignore(const collide::ray_defs&, CObject* object, LPVOID)
	{
		return (object->spatial.type & STYPE_FEELVISIONIGNORE) ? FALSE : TRUE;
	}

	collide::test_callback* vision_test_callback() { return feel_vision_ignore; }
#else
	collide::test_callback* vision_test_callback() { return 0; }
#endif

	void ai_visible_hits(const segment& seg, float range, const CObject* ignore, const CObject* target,
	                     bool drawn_only, xr_vector<body_hit>& out)
	{
		static xr_vector<ISpatial*> spatial;
		static collide::rq_results rq;

		out.clear();
		spatial.clear_not_free();
		g_SpatialSpace->q_ray(spatial, 0, STYPE_VISIBLEFORAI, seg.from, seg.dir, range);

		const collide::ray_defs rd(seg.from, seg.dir, range, CDB::OPT_ONLYFIRST,
		                           collide::rq_target(collide::rqtStatic | collide::rqtObject | collide::rqtObstacle));
		for (u32 i = 0; i < spatial.size(); ++i)
		{
			CObject* O = spatial[i]->dcast_CObject();
			if (O && (O == ignore || O == target))
				continue;
#ifdef SPATIAL_CHANGE
			if (O && (O->spatial.type & STYPE_FEELVISIONIGNORE))
				continue;
#endif
			body_hit h;
			h.O = O;
			h.drawn = O && O->Visual() && O->getVisible();
			if (drawn_only && !h.drawn)
				continue;
			h.sphere = !O || !O->collidable.model;
			if (h.sphere)
			{
				Fvector c;
				c.sub(spatial[i]->spatial.sphere.P, seg.from);
				h.range = _max(0.f, c.dotproduct(seg.dir));
			}
			else
			{
				rq.r_clear();
				if (!O->collidable.model->_RayQuery(rd, rq) || !rq.r_count())
					continue;
				h.range = rq.r_begin()->range;
			}
			out.push_back(h);
		}
		std::sort(out.begin(), out.end(), [](const body_hit& l, const body_hit& r) { return l.range < r.range; });
	}

	bool occlusion_query::trace(const segment& seg, u32 rq_flags, CObject* ignore, const CObject* target,
	                            occlusion_result& out, const trace_options& opt) const
	{
		out.reset();
		if (!m_ready)
			return false;

		// A recorded trace is what the debug view draws, so it is always a full one.
		const static_half* reuse = opt.reuse;
		float clear_threshold = opt.clear_threshold;
		float visible_min = opt.visible_min;
		bool materials_only = opt.materials_only;
#ifdef PERCEPTION_DEBUG
		if (record)
		{
			reuse = 0;
			clear_threshold = 0.f;
			visible_min = 0.f;
			materials_only = false;
		}
#endif

		// Atmospheric extinction: O(1), no ray, no cache - a table lookup against the current fog
		// envelope. It is the only channel that is a property of the path rather than of what lies
		// along it.
		out.channel[occ_atmosphere] = m_model.atmosphere_transmittance(atmosphere::instance().transmittance(seg));
		if (atmosphere::instance().unmeasured())
			out.degraded |= deg_atmo_unmeasured;
		const bool fog_hides = visible_min > 0.f && out.channel[occ_atmosphere] < visible_min;

		// A half that skipped grass behind fog or canopy says nothing about the grass once they no longer
		// hide the point.
		if (reuse && !reuse->grass_marched && !(reuse->static_materials < clear_threshold))
		{
			const float canopy = reuse->canopy_marched ? m_model.canopy_transmittance(reuse->canopy_tau) : 1.f;
			if (!(visible_min > 0.f && out.channel[occ_atmosphere] * canopy < visible_min))
				reuse = 0;
		}
		out.refreshed = 0 != reuse;

		// Level geometry does not move, so a segment through the front face of a triangle that blocked
		// before is blocked by it now. The query below culls the same way.
#ifdef PERCEPTION_DEBUG
		bool tri_check = false;
#endif
		if (opt.block_tri && !reuse && clear_threshold > 0.f)
		{
			Fvector tri[3] = {opt.block_tri[0], opt.block_tri[1], opt.block_tri[2]};
			float u, v, range;
			if (CDB::TestRayTri(seg.from, seg.dir, tri, u, v, range, true) && range > 0.f && range < seg.length)
			{
#ifdef PERCEPTION_DEBUG
				++ray_cost.tri_hits;
				tri_check = 0 == ray_cost.tri_hits % TRI_CHECK_EVERY;
				if (!tri_check)
#endif
				{
					out.channel[occ_hard] = 0.f;
					out.level.geom[occ_hard] = 0.f;
					out.level.static_materials = 0.f;
					out.level.reusable = true;
					out.tri_blocked = true;
					out.concealment_skipped = true;
					out.hit_count = 1;
					out.finish();
					return true;
				}
			}
		}

		float cached_depth = 0.f;
		bool got_depth = false, got_geom = false;
		float cached_geom[occ_class_count];
		if (!reuse)
			m_cache.lookup(seg.from, seg.to, cached_depth, got_depth, cached_geom, got_geom);

		// Kept apart from out.channel[] because only the level-static contribution may be cached;
		// folding the dynamic hits in here would poison every later reader of that entry.
		float static_geom[occ_class_count];
		for (u32 i = 0; i < occ_class_count; ++i)
			static_geom[i] = 1.f;

		out.from_cache_geom = got_geom;

		// The ray is cast whether or not the geometry half hit, and the cached value is deliberately
		// NOT consumed in this stage. Two reasons. The static CDB trace is an optimised AABB walk, so
		// skipping it saves little next to the foliage march that IS cached; and serving it from cache
		// would leave hit_count, dominant_material and the drawn hit points describing only the
		// dynamic half, so a cached trace would report differently from an identical uncached one -
		// exactly the asymmetry that makes a debug view lie. So got_geom stays a pure measurement of
		// "would this have hit", which is the number Stage 2 exists to produce, and comparing it
		// against the traced answer below costs nothing because both are already in hand.
		//
		// Passing a null callback returns the whole hit list sorted near->far
		// (xr_area_raypick.cpp:265-277), which keeps accumulation a plain pass over the list instead
		// of a void*-threaded early-out. Same target set as v1 (Feel_Vision.cpp:218) - notably without
		// rqtShape, which would pick up space restrictors and other non-visual volumes.
		const collide::rq_target dyn = collide::rq_target(collide::rqtObject | collide::rqtObstacle);
		const collide::rq_target tgt = collide::rq_target(collide::rqtStatic | dyn);

		const collide::ray_defs rd = seg.to_ray_defs(rq_flags, reuse ? dyn : tgt);
#ifdef PERCEPTION_DEBUG
		CTimer query_timer;
		query_timer.Start();
#endif
		Level().ObjectSpace.RayQuery(m_hits, rd, 0, 0, vision_test_callback(), ignore);
#ifdef PERCEPTION_DEBUG
		++ray_cost.queries;
		ray_cost.query_us += query_timer.GetElapsed_sec() * 1000000.0;
#endif

		float worst = 1.f;
		const float limit = seg.length;

#ifdef PERCEPTION_DEBUG
		if (record)
			last_hits.clear();
#endif

		// A material mapped to occ_canopy is a vegetation proxy hull whose real leaf cards are already
		// in the canopy field, so its hit is dropped rather than multiplied in. Without a field there
		// is nothing to defer to and the hull is the only vegetation the ray can see, so it falls back
		// to soft - a level or mod with proxies but no leaf geometry must not lose occlusion entirely.
		//
		// Gated on complete(), not ready(): a bush whose leaf atlas could not be read gets no field
		// cells, so dropping its proxy hull too would leave it with no occlusion at all - a
		// false-clear, which is the one direction this feature exists to prevent. When the field is
		// incomplete both contribute; double-counting over-occludes, which is the safe direction.
		const bool canopy_ready = canopy_oracle::instance().complete();

		// r_begin/r_end dereference vector iterators (xr_collide_defs.h:150-151), so an empty result
		// set must not reach them.
		if (m_hits.r_count())
		{
			for (collide::rq_result* h = m_hits.r_begin(); h != m_hits.r_end(); ++h)
			{
				if (h->range > limit)
					break; // sorted, so nothing past here is on the segment either

				if (target && h->O == target)
				{
					if (!out.self_hits)
						out.self_range = h->range;
					++out.self_hits;
					continue;
				}

				const u16 mtl = material_of(*h);
				occluder_class c = classify(*h, mtl, m_table);
				const float t = material_transmittance(h->O, h->element);
				if (!h->O && occ_canopy != c && !out.has_blocker && fis_zero(t))
				{
					const CDB::TRI* T = Level().ObjectSpace.GetStaticTris() + h->element;
					const Fvector* V = Level().ObjectSpace.GetStaticVerts();
					for (u32 k = 0; k < 3; ++k)
						out.blocker[k] = V[T->verts[k]];
					out.has_blocker = true;
				}
				if (occ_canopy == c)
				{
					if (canopy_ready)
					{
						++out.deferred_hits;
						out.deferred_t *= t;
						if (!h->O)
						{
							++out.level.deferred_hits;
							out.level.deferred_t *= t;
						}
						continue;
					}
					++out.fallback_hits;
					if (!h->O)
						++out.level.fallback_hits;
					c = occ_soft;
				}

				out.channel[c] *= t;
				if (!h->O)
					static_geom[c] *= t;
				++out.hit_count;

#ifdef PERCEPTION_DEBUG
				if (record)
				{
					hit_mark hm;
					hm.pos.mad(seg.from, seg.dir, h->range);
					hm.cls = c;
					hm.t = t;
					last_hits.push_back(hm);
				}
#endif

				if (t < worst)
				{
					worst = t;
					const SGameMtl* m = GMLib.GetMaterialByIdx(mtl);
					if (m)
						out.dominant_material = m->m_Name;
				}
			}
		}

		if (reuse)
		{
			for (u32 i = 0; i < occ_class_count; ++i)
				out.channel[i] *= reuse->geom[i];
			out.deferred_t *= reuse->deferred_t;
			out.deferred_hits = u16(out.deferred_hits + reuse->deferred_hits);
			out.fallback_hits = u16(out.fallback_hits + reuse->fallback_hits);
			out.level = *reuse;
		}
		else
		{
			for (u32 i = 0; i < occ_class_count; ++i)
				out.level.geom[i] = static_geom[i];
		}

		// v1 zeroes the ray on any of these, whatever its material (Feel_Vision.cpp:294-295). They can
		// only zero a channel, so behind a material block they change nothing.
		const bool blocked = out.channel[occ_hard] * out.channel[occ_soft] * out.channel[occ_creature] < clear_threshold;
#ifdef PERCEPTION_DEBUG
		if (blocked)
			++ray_cost.bodies_skipped;
		else
			++ray_cost.bodies_run;
		CTimer bodies_timer;
		bodies_timer.Start();
#endif
		if (blocked)
			m_bodies.clear();
		else
			ai_visible_hits(seg, seg.length, ignore, target, true, m_bodies);
#ifdef PERCEPTION_DEBUG
		if (!blocked)
			ray_cost.bodies_us += bodies_timer.GetElapsed_sec() * 1000000.0;
#endif
		for (u32 i = 0; i < m_bodies.size(); ++i)
		{
			const body_hit& b = m_bodies[i];
			const occluder_class c = b.O && smart_cast<const CEntityAlive*>(b.O) ? occ_creature : occ_hard;
			out.channel[c] = 0.f;
			++out.body_hits;
			if (b.sphere)
				++out.sphere_hits;
			++out.hit_count;

#ifdef PERCEPTION_DEBUG
			if (record)
			{
				hit_mark hm;
				hm.pos.mad(seg.from, seg.dir, b.range);
				hm.cls = c;
				hm.t = 0.f;
				last_hits.push_back(hm);
			}
#endif
		}

#ifdef PERCEPTION_DEBUG
		if (got_geom)
		{
			float e = 0.f;
			for (u32 i = 0; i < occ_class_count; ++i)
				e = _max(e, _abs(cached_geom[i] - static_geom[i]));
			++validate_n;
			validate_sum += e;
			validate_max = _max(validate_max, e);
		}
#endif

		// v_p is 0 behind a material block, and below visible_min once fog, then fog times canopy, alone
		// fall under it, whatever the rest holds (FEED-DESIGN 2). Canopy is marched before grass
		// (FEED-DESIGN 3.6).
		const float materials = out.channel[occ_hard] * out.channel[occ_soft] * out.channel[occ_creature];
		const bool material_block = materials < clear_threshold;
#ifdef PERCEPTION_DEBUG
		if (tri_check)
		{
			++ray_cost.tri_checked;
			if (!material_block)
				++ray_cost.tri_wrong;
		}
#endif
		const bool conceal = !reuse && !materials_only && !material_block && !fog_hides;
		out.concealment_skipped = !reuse && !conceal;

#ifdef PERCEPTION_DEBUG
		if (!reuse && !materials_only)
		{
			if (material_block)
				++march_cost.material_skipped;
			else if (fog_hides)
				++march_cost.fog_skipped;
		}
		if (record)
		{
			last_chords.clear();
			last_canopy.clear();
			last_canopy_hits.clear();
		}
		CTimer march_timer;
		march_timer.Start();
#endif

		if (reuse)
		{
			out.foliage_tau = reuse->foliage_tau;
			out.canopy_tau = reuse->canopy_tau;
			out.canopy_crowns = reuse->canopy_crowns;
			out.degraded |= reuse->degraded;
			if (reuse->canopy_marched)
				out.channel[occ_canopy] = m_model.canopy_transmittance(reuse->canopy_tau);
			if (reuse->grass_marched)
				out.channel[occ_foliage] = m_model.foliage_transmittance(reuse->foliage_tau);
		}

		// Canopy is not cached and has no budget: every crown is resident, so the march is a column walk
		// with nothing to resolve and nothing to defer.
		if (conceal)
		{
			canopy_oracle::march_result cm;
			cm.tau = 0.f;
			cm.crowns = 0;

			// The near plane follows the renderer's: a card the player's camera would clip is not drawn.
			canopy_oracle::sight_cone cone;
			cone.eye = m_model.canopy_cone_eye;
			cone.target = m_model.canopy_cone_target;
			cone.near_plane = VIEWPORT_NEAR;
#ifdef PERCEPTION_DEBUG
			canopy_oracle::instance().march(seg.from, seg.to, cone, cm, record ? &last_canopy : 0,
			                                record ? &last_canopy_hits : 0);
#else
			canopy_oracle::instance().march(seg.from, seg.to, cone, cm);
#endif
			out.canopy_tau = cm.tau;
			out.canopy_crowns = cm.crowns;

			// Incomplete means crowns may be missing anywhere, so every trace is flagged, not only
			// those that crossed a cell.
			const canopy_oracle& co = canopy_oracle::instance();
			if (co.ready() && !co.complete())
				out.degraded |= deg_canopy_incomplete;
			if (cm.crowns && co.overcounted())
				out.degraded |= deg_canopy_overcount;
			out.channel[occ_canopy] = m_model.canopy_transmittance(cm.tau);
		}

		const bool canopy_hides = visible_min > 0.f && out.channel[occ_atmosphere] * out.channel[occ_canopy] < visible_min;
		const bool grass_marched = conceal && !canopy_hides;

#ifdef PERCEPTION_DEBUG
		const float canopy_us = march_timer.GetElapsed_sec() * 1000000.f;
		const float resolve_us_before = foliage_oracle::instance().get_stats().resolve_us;
		march_timer.Start();
#endif

		if (grass_marched)
		{
			foliage_oracle::march_result fm;
			fm.tau = 0.f;
			fm.crossings = fm.unresolved = 0;

#ifdef PERCEPTION_DEBUG
			const bool want_chords = record;
#else
			const bool want_chords = false;
#endif

			// A cached tau is only reusable when nothing else needs the march. Recording chords forces
			// the real walk, which is correct: the debug view must show what is actually there.
			//
			// A march that hit the resolve budget is NOT cached: it is a deliberate over-estimate for
			// this frame only, and storing it would make a transient budget miss permanent.
			if (got_depth && !want_chords)
			{
				fm.tau = cached_depth;
				out.from_cache_depth = true;
			}
			else
			{
#ifdef PERCEPTION_DEBUG
				foliage_oracle::instance().march(seg.from, seg.to, fm, want_chords ? &last_chords : 0);
#else
				foliage_oracle::instance().march(seg.from, seg.to, fm, 0);
#endif
			}

			out.foliage_tau = fm.tau;
			out.foliage_unresolved = fm.unresolved;
			if (fm.unresolved)
				out.degraded |= deg_foliage_unresolved;
			out.channel[occ_foliage] = m_model.foliage_transmittance(fm.tau);
		}

#ifdef PERCEPTION_DEBUG
		if (conceal)
		{
			march_stats& ms = march_cost;
			++ms.traces;
			ms.canopy_us += canopy_us;
			if (grass_marched)
			{
				++ms.grass_traces;
				ms.grass_us += march_timer.GetElapsed_sec() * 1000000.f;
				ms.grass_resolve_us += foliage_oracle::instance().get_stats().resolve_us - resolve_us_before;
				if (out.foliage_unresolved)
					++ms.unresolved;
			}
			else
				++ms.grass_skipped;
		}
#endif

		if (!reuse)
		{
			out.level.foliage_tau = out.foliage_tau;
			out.level.canopy_tau = out.canopy_tau;
			out.level.canopy_crowns = out.canopy_crowns;
			out.level.degraded = u8(out.degraded & (deg_canopy_incomplete | deg_canopy_overcount));
			out.level.canopy_marched = conceal;
			out.level.grass_marched = grass_marched;

			// Skipped behind a block of its own static geometry, the half stays blocked whatever moves;
			// skipped behind a creature, it says nothing about the grass once the creature has gone.
			// Skipped behind fog or canopy, it holds while they still hide the point, which a reuse
			// re-checks.
			out.level.static_materials = static_geom[occ_hard] * static_geom[occ_soft] * static_geom[occ_creature];
			const bool static_block = out.level.static_materials < clear_threshold;
			out.level.reusable = grass_marched ? (0 == out.foliage_unresolved)
			                                   : (static_block || (!material_block && (fog_hides || canopy_hides)));
		}

		out.finish();

		if (!grass_marched)
			return true;

		static_occlusion_cache::entry e;
		e.from = seg.from;
		e.to = seg.to;
		e.foliage_tau = out.foliage_tau;
		for (u32 i = 0; i < occ_class_count; ++i)
			e.geom[i] = static_geom[i];
		e.tau_valid = (0 == out.foliage_unresolved);
		e.stamp = Device.dwFrame;
		e.used = true;
		m_cache.store(e);

#ifdef PERCEPTION_DEBUG
		float check = 1.f;
		for (u32 i = 0; i < occ_class_count; ++i)
			check *= out.channel[i];
		VERIFY(fsimilar(check, out.transmittance, EPS_L));
#endif
		return true;
	}

	namespace occlusion
	{
		void on_level_load()
		{
			atmosphere::instance().on_level_load();
			occlusion_query::instance().on_level_load();
			vision_feed::on_level_load();
			PDBG_ONLY(shadow::on_level_load());
		}

		void on_level_unload()
		{
			PDBG_ONLY(shadow::on_level_unload());
			occlusion_query::instance().on_level_unload();
			atmosphere::instance().on_level_unload();
		}
	}
}
