#include "stdafx.h"
#include "occlusion_query.h"
#include "atmosphere.h"
#include "canopy_oracle.h"
#include "perception_debug.h"
#include "../Level.h"
#include "../entity_alive.h"
#include "../../Include/xrRender/Kinematics.h"
#include "../../xrEngine/GameMtlLib.h"

namespace perception
{
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

	static u16 material_of(const collide::rq_result& h)
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
		last_hits.clear();
		record = false;
		reset_validation();
#endif
		m_ready = false;
	}

	bool occlusion_query::trace(const segment& seg, u32 rq_flags, CObject* ignore,
	                            occlusion_result& out) const
	{
		return trace_impl(seg, rq_flags, ignore, out);
	}

	bool occlusion_query::trace_head(const segment& head, u32 rq_flags, CObject* ignore,
	                                 occlusion_result& out) const
	{
		return trace_impl(head, rq_flags, ignore, out);
	}

	bool occlusion_query::trace_tail(const occlusion_result& head_result, const segment& tail,
	                                 u32 rq_flags, CObject* ignore, occlusion_result& out) const
	{
		if (!trace_impl(tail, rq_flags, ignore, out))
			return false;

		// Channels compose multiplicatively, which is the whole reason they are transmittance rather
		// than occlusion - the prefix and the tail simply multiply. Atmosphere is the exception: it
		// is a function of the whole path, and for a normalized-distance curve T(a+b) != T(a)*T(b)
		// (stock linear gives (1-a)(1-b) = 1-a-b+ab, not 1-(a+b)). So it is recomputed below.
		for (u32 i = 0; i < occ_class_count; ++i)
		{
			if (i == occ_atmosphere)
				continue;
			out.channel[i] *= head_result.channel[i];
		}

		out.total_length = head_result.total_length + tail.length;
		out.channel[occ_atmosphere] = m_model.atmosphere_transmittance(atmosphere::instance().transmittance(
			out.total_length, atmosphere::resolve_height(head_result.head_y, tail.to.y)));

		out.foliage_tau += head_result.foliage_tau;
		out.canopy_tau += head_result.canopy_tau;
		out.canopy_cells = u16(out.canopy_cells + head_result.canopy_cells);
		out.foliage_unresolved += head_result.foliage_unresolved;
		out.deferred_hits = u16(out.deferred_hits + head_result.deferred_hits);
		out.deferred_t *= head_result.deferred_t;
		out.fallback_hits = u16(out.fallback_hits + head_result.fallback_hits);
		out.degraded |= head_result.degraded;
		out.hit_count = u16(out.hit_count + head_result.hit_count);
		if (head_result.dominant_material.size() && !out.dominant_material.size())
			out.dominant_material = head_result.dominant_material;

		out.finish();
		return true;
	}

	bool occlusion_query::trace_impl(const segment& seg, u32 rq_flags, CObject* ignore,
	                                 occlusion_result& out) const
	{
		out.reset();
		if (!m_ready)
			return false;

		float cached_depth = 0.f;
		bool got_depth = false, got_geom = false;
		float cached_geom[occ_class_count];
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

		const collide::ray_defs rd = seg.to_ray_defs(rq_flags, tgt, m_model.ray_extend);
		Level().ObjectSpace.RayQuery(m_hits, rd, 0, 0, 0, ignore);

		float worst = 1.f;
		const float limit = seg.length + m_model.ray_extend;

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

				const u16 mtl = material_of(*h);
				occluder_class c = classify(*h, mtl, m_table);
				const float t = material_transmittance(h->O, h->element);
				if (occ_canopy == c)
				{
					if (canopy_ready)
					{
						++out.deferred_hits;
						out.deferred_t *= t;
						continue;
					}
					++out.fallback_hits;
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

		{
			foliage_oracle::march_result fm;
			fm.tau = 0.f;
			fm.crossings = fm.unresolved = 0;

#ifdef PERCEPTION_DEBUG
			const bool want_chords = record;
			if (want_chords)
				last_chords.clear();
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

		// Canopy is not cached and has no budget: the whole field is resident, so the march is a grid
		// walk with nothing to resolve and nothing to defer.
		{
			canopy_oracle::march_result cm;
			cm.tau = 0.f;
			cm.cells = 0;
#ifdef PERCEPTION_DEBUG
			if (record)
				last_canopy.clear();
			canopy_oracle::instance().march(seg.from, seg.to, cm, record ? &last_canopy : 0);
#else
			canopy_oracle::instance().march(seg.from, seg.to, cm, 0);
#endif
			out.canopy_tau = cm.tau;
			out.canopy_cells = cm.cells;

			// Incomplete means crowns may be missing anywhere, so every trace is flagged, not only
			// those that crossed a cell.
			const canopy_oracle& co = canopy_oracle::instance();
			if (co.ready() && !co.complete())
				out.degraded |= deg_canopy_incomplete;
			if (cm.cells && co.overcounted())
				out.degraded |= deg_canopy_overcount;
			out.channel[occ_canopy] = m_model.canopy_transmittance(cm.tau);
		}

		// Atmospheric extinction: O(1), no ray, no cache - a table lookup against the current fog
		// envelope. It is the only channel that is a property of the path rather than of what lies
		// along it, which is why trace_tail recomputes rather than multiplies it.
		out.total_length = seg.length;
		out.head_y = seg.from.y;
		out.channel[occ_atmosphere] = m_model.atmosphere_transmittance(atmosphere::instance().transmittance(seg));
		if (atmosphere::instance().unmeasured())
			out.degraded |= deg_atmo_unmeasured;

		out.finish();

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
		}

		void on_level_unload()
		{
			occlusion_query::instance().on_level_unload();
			atmosphere::instance().on_level_unload();
		}
	}
}
