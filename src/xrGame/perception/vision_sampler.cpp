#include "stdafx.h"
#include "vision_sampler.h"
#include "occlusion_query.h"
#include "bone_shape.h"
#include "../../Include/xrRender/Kinematics.h"
#include "../../xrEngine/bone.h"

namespace perception
{
	float vision_sampler::s_current_eps = 0.10f;
	u32 vision_sampler::s_refresh_visits = 4;

	float point_visibility(const float channel[occ_class_count], float clear_threshold)
	{
		const float materials = channel[occ_hard] * channel[occ_soft] * channel[occ_creature];
		if (materials < clear_threshold)
			return 0.f;
		return channel[occ_foliage] * channel[occ_canopy] * channel[occ_atmosphere];
	}

	bool point_fails(float v, float visible_min)
	{
		return !(v > 0.f) || v < visible_min;
	}

	occluder_class lowest_channel(const float channel[occ_class_count])
	{
		occluder_class lowest = occluder_class(occ_class_count);
		float worst = 1.f - EPS_L;
		for (u32 c = 0; c < occ_class_count; ++c)
		{
			if (channel[c] < worst)
			{
				worst = channel[c];
				lowest = occluder_class(c);
			}
		}
		return lowest;
	}

	vision_sampler::vision_sampler(s32 seed) : m_rnd(seed)
	{
	}

	void vision_sampler::configure(float current_eps, u32 refresh_visits)
	{
		s_current_eps = current_eps;
		s_refresh_visits = refresh_visits;
	}

	vision_sampler::pair_state* vision_sampler::find(u16 id)
	{
		for (u32 i = 0; i < m_pairs.size(); ++i)
			if (m_pairs[i].id == id)
				return &m_pairs[i];
		return 0;
	}

	const vision_sampler::pair_state* vision_sampler::find(u16 id) const
	{
		for (u32 i = 0; i < m_pairs.size(); ++i)
			if (m_pairs[i].id == id)
				return &m_pairs[i];
		return 0;
	}

	void vision_sampler::build_slots(const CObject* target, pair_state& p)
	{
		p.slots.clear();
		p.records.clear();
		p.cursor = 0;
		p.visual = target->Visual();
		p.sticky_set = false;
		p.sticky_slot = 0;
		ZeroMemory(&p.sticky, sizeof(p.sticky));
		p.has_block_tri = false;

		IKinematics* K = smart_cast<IKinematics*>(target->Visual());
		if (!K)
			return;

		p.bones_visible = K->LL_GetBonesVisible();

		// Same population as CEntityAlive::fill_hit_bone_surface_areas.
		for (u16 i = 0, n = K->LL_BoneCount(); i < n; ++i)
		{
			const SBoneShape& shape = K->LL_GetData(i).shape;
			if (SBoneShape::stNone == shape.type || shape.flags.is(SBoneShape::sfNoPickable))
				continue;

			sample_slot s;
			s.bone = i;
			s.weight = bone_shape_area(shape);
			p.slots.push_back(s);
		}

		slot_record blank;
		ZeroMemory(&blank, sizeof(blank));
		p.records.assign(p.slots.size(), blank);
	}

	vision_sampler::pair_state& vision_sampler::acquire(const CObject* target)
	{
		pair_state* p = find(target->ID());
		if (!p)
		{
			m_pairs.push_back(pair_state());
			p = &m_pairs.back();
			p->id = target->ID();
			p->object = 0;
		}

		if (p->object != target || p->visual != target->Visual())
		{
			p->object = target;
			build_slots(target, *p);
		}
		return *p;
	}

	bool vision_sampler::current(const CObject* target, const slot_record& rec, u16 bone, const Fvector& eye) const
	{
		if (!rec.time)
			return false;
		const float eps2 = s_current_eps * s_current_eps;
		const Fvector world = target->get_last_local_point_on_mesh(rec.local, bone);
		return eye.distance_to_sqr(rec.eye) <= eps2 && world.distance_to_sqr(rec.point) <= eps2;
	}

	// v1's draw: a visible slot picked by area, then a random point on its shape
	// (CEntityAlive::get_new_local_point_on_mesh).
	void vision_sampler::draw_sticky(pair_state& p, IKinematics* K)
	{
		float total = 0.f;
		for (u32 i = 0; i < p.slots.size(); ++i)
			if (K->LL_GetBoneVisible(p.slots[i].bone))
				total += p.slots[i].weight;

		p.sticky_set = false;
		if (total <= 0.f)
			return;

		float pick = m_rnd.randF(total);
		u32 slot = u32(-1);
		for (u32 i = 0; i < p.slots.size(); ++i)
		{
			if (!K->LL_GetBoneVisible(p.slots[i].bone))
				continue;
			slot = i;
			pick -= p.slots[i].weight;
			if (pick < 0.f)
				break;
		}

		ZeroMemory(&p.sticky, sizeof(p.sticky));
		p.sticky.local = random_point_on_bone_shape(K->LL_GetData(p.slots[slot].bone).shape, m_rnd);
		p.sticky_slot = slot;
		p.sticky_set = true;
	}

	bool vision_sampler::visit(pair_state& p, CObject* observer, const Fvector& eye, const CObject* target,
	                           IKinematics* K, u16 bone, slot_record& rec, bool keep_point, u32 now,
	                           const trace_options& limits, sample_counts* counts, trace_listener* listener)
	{
		const float eps2 = s_current_eps * s_current_eps;
		Fvector local = rec.local;
		Fvector world = target->get_last_local_point_on_mesh(local, bone);

		full_reason why = full_reason_count;
		if (!rec.time)
			why = full_new_point;
		else if (!rec.level.reusable)
			why = full_not_reusable;
		else if (eye.distance_to_sqr(rec.eye) > eps2)
			why = full_eye_moved;
		else if (world.distance_to_sqr(rec.point) > eps2)
			why = full_point_moved;
		else if (rec.refreshes + 1 >= s_refresh_visits)
			why = full_visit_limit;
		const bool refresh = full_reason_count == why;

		if (!refresh && !keep_point)
		{
			local = random_point_on_bone_shape(K->LL_GetData(bone).shape, m_rnd);
			world = target->get_last_local_point_on_mesh(local, bone);
		}

		segment seg;
		if (!segment::make(eye, world, seg))
			return false;

		trace_options opt = limits;
		opt.reuse = refresh ? &rec.level : 0;
		opt.block_tri = p.has_block_tri ? p.block_tri : 0;

		occlusion_result r;
		if (!occlusion_query::instance().trace(seg, CDB::OPT_CULL, observer, target, r, opt))
			return false;

		if (r.has_blocker)
		{
			for (u32 k = 0; k < 3; ++k)
				p.block_tri[k] = r.blocker[k];
			p.has_block_tri = true;
		}

		if (counts)
		{
			if (r.refreshed)
				++counts->refreshed;
			else
				++counts->full[refresh ? full_not_reusable : why];
		}

		if (!r.refreshed)
		{
			rec.eye = eye;
			rec.point = world;
			rec.local = local;
			rec.level = r.level;
			rec.refreshes = 0;
		}
		else
			++rec.refreshes;
		for (u32 c = 0; c < occ_class_count; ++c)
			rec.channel[c] = r.channel[c];
		rec.t = r.transmittance;
		rec.t_v1 = r.v1_transmittance();
		rec.v = point_visibility(rec.channel, limits.clear_threshold);
		rec.dominant = r.dominant;
		rec.degraded = r.degraded;
		rec.self_occluded = r.self_occluded(seg.length);
		rec.time = now;
		++rec.visits;

		if (listener)
			listener->on_trace(seg, bone, r);
		return true;
	}

	void vision_sampler::sample(CObject* observer, const Fvector& eye, const CObject* target, u32 k, u32 now,
	                            const trace_options& limits, sample_counts* counts, trace_listener* listener)
	{
		const occlusion_query& q = occlusion_query::instance();
		if (!q.ready() || !target || !k)
			return;

		pair_state& p = acquire(target);

		const u32 n = p.slots.size();
		if (!n)
			return;

		IKinematics* K = smart_cast<IKinematics*>(target->Visual());
		VERIFY(K);

		const u64 visible = K->LL_GetBonesVisible();
		if (visible != p.bones_visible)
		{
			p.bones_visible = visible;
			for (u32 i = 0; i < n; ++i)
				p.records[i].visits = 0;
		}

		if (p.sticky_set && !K->LL_GetBoneVisible(p.slots[p.sticky_slot].bone))
			p.sticky_set = false;
		else if (p.sticky_set && p.sticky.time && point_fails(p.sticky.v, limits.visible_min))
			reroll_sticky(p, K, target, eye, limits.visible_min, counts);
		if (!p.sticky_set)
			draw_sticky(p, K);
		if (p.sticky_set)
			visit(p, observer, eye, target, K, p.slots[p.sticky_slot].bone, p.sticky, true, now, limits, counts,
			      listener);

		const float eps2 = s_current_eps * s_current_eps;
		u32 traced = 0;
		for (u32 tried = 0; tried < n && traced + 1 < k; ++tried)
		{
			const u32 i = p.cursor;
			p.cursor = (p.cursor + 1) % n;

			const sample_slot& s = p.slots[i];
			slot_record& rec = p.records[i];

			// Hidden bones (dismemberment, outfit parts) drop out, as they do from v1's draw.
			if (!K->LL_GetBoneVisible(s.bone))
			{
				rec.time = 0;
				continue;
			}

			// Moving objects only ever add occlusion, so a still point behind its own static geometry
			// stays at 0 until an end moves. It costs no ray and no share of k.
			if (rec.time && rec.level.static_materials < limits.clear_threshold && eye.distance_to_sqr(rec.eye) <= eps2 &&
			    target->get_last_local_point_on_mesh(rec.local, s.bone).distance_to_sqr(rec.point) <= eps2)
			{
				rec.time = now;
				++rec.visits;
				if (counts)
					++counts->skipped_static;
				continue;
			}

			if (visit(p, observer, eye, target, K, s.bone, rec, false, now, limits, counts, listener))
				++traced;
		}
	}

	// The best visible current rotating record, else v1's draw.
	void vision_sampler::reroll_sticky(pair_state& p, IKinematics* K, const CObject* target, const Fvector& eye,
	                                   float visible_min, sample_counts* counts)
	{
		const u32 n = p.slots.size();
		u32 best = u32(-1);
		float best_v = 0.f;
		for (u32 i = 0; i < n; ++i)
		{
			const slot_record& rec = p.records[i];
			if (point_fails(rec.v, visible_min) || rec.v <= best_v || !K->LL_GetBoneVisible(p.slots[i].bone) ||
			    !current(target, rec, p.slots[i].bone, eye))
				continue;
			best = i;
			best_v = rec.v;
		}

		if (best != u32(-1))
		{
			p.sticky = p.records[best];
			p.sticky_slot = best;
			if (counts)
				++counts->rerolls_best;
		}
		else
		{
			draw_sticky(p, K);
			if (counts)
				++counts->rerolls_fresh;
		}
	}

	bool vision_sampler::summary(const CObject* target, const Fvector& eye, float clear_threshold, float visible_min,
	                             u32 now, pair_summary& out) const
	{
		ZeroMemory(&out, sizeof(out));
		out.limiting = occluder_class(occ_class_count);
		out.sticky_dominant = occluder_class(occ_class_count);

		const pair_state* p = target ? find(target->ID()) : 0;
		if (!p || p->object != target || p->slots.empty())
			return false;

		IKinematics* K = smart_cast<IKinematics*>(target->Visual());
		if (!K)
			return false;

		float w = 0.f, clear = 0.f, clear_v1 = 0.f, clear_v1_self = 0.f, self = 0.f, t = 0.f;
		float cur_w = 0.f, cur_v = 0.f;
		u32 best_rot = u32(-1);
		float channel[occ_class_count];
		for (u32 c = 0; c < occ_class_count; ++c)
			channel[c] = 0.f;

		u32 oldest = 0, newest = u32(-1), vmin = u32(-1), vmax = 0;
		for (u32 i = 0; i < p->slots.size(); ++i)
		{
			if (!K->LL_GetBoneVisible(p->slots[i].bone))
				continue;
			++out.slots;

			const slot_record& rec = p->records[i];
			vmin = _min(vmin, rec.visits);
			vmax = _max(vmax, rec.visits);
			if (!rec.time)
				continue;
			++out.sampled;

			const float sw = p->slots[i].weight;
			w += sw;
			if (rec.t >= clear_threshold) clear += sw;
			if (rec.t_v1 >= clear_threshold) clear_v1 += sw;
			if (rec.t_v1 >= clear_threshold && !rec.self_occluded) clear_v1_self += sw;
			if (rec.self_occluded) self += sw;
			t += sw * rec.t;
			for (u32 c = 0; c < occ_class_count; ++c)
				channel[c] += sw * rec.channel[c];

			out.degraded |= rec.degraded;
			const u32 age = now - rec.time;
			oldest = _max(oldest, age);
			newest = _min(newest, age);

			if (current(target, rec, p->slots[i].bone, eye))
			{
				++out.current;
				cur_w += sw;
				cur_v += sw * rec.v;
				if (u32(-1) == best_rot || rec.v > p->records[best_rot].v)
					best_rot = i;
				out.best = _max(out.best, rec.v);
			}
		}

		const bool sticky_current = p->sticky_set && K->LL_GetBoneVisible(p->slots[p->sticky_slot].bone) &&
			current(target, p->sticky, p->slots[p->sticky_slot].bone, eye);
		if (sticky_current)
		{
			out.has_sticky = true;
			out.best = _max(out.best, p->sticky.v);
			out.sticky_dominant = lowest_channel(p->sticky.channel);
		}
		out.seen = !point_fails(out.best, visible_min);

		const slot_record* point = 0;
		u32 point_slot = 0;
		if (sticky_current && !point_fails(p->sticky.v, visible_min))
		{
			point = &p->sticky;
			point_slot = p->sticky_slot;
		}
		else if (u32(-1) != best_rot && !point_fails(p->records[best_rot].v, visible_min))
		{
			point = &p->records[best_rot];
			point_slot = best_rot;
		}
		else if (p->sticky_set)
		{
			point = &p->sticky;
			point_slot = p->sticky_slot;
		}
		if (point)
		{
			out.has_point = true;
			out.point_bone = p->slots[point_slot].bone;
			out.point_local = point->local;
		}
		out.visibility = cur_w > 0.f ? cur_v / cur_w : (sticky_current ? p->sticky.v : 0.f);

		if (out.sampled && w > 0.f)
		{
			out.weight_total = w;
			out.coverage = clear / w;
			out.coverage_v1 = clear_v1 / w;
			out.coverage_v1_self = clear_v1_self / w;
			out.self_share = self / w;
			out.transmittance = t / w;
			out.oldest_age = oldest;
			out.newest_age = newest;
			out.visit_spread = vmax - vmin;

			float worst = flt_max;
			for (u32 c = 0; c < occ_class_count; ++c)
			{
				const float mean = channel[c] / w;
				if (mean < worst && mean < 1.f - EPS_L)
				{
					worst = mean;
					out.limiting = occluder_class(c);
				}
			}
		}
		return out.has_sticky || out.sampled;
	}

	const xr_vector<slot_record>* vision_sampler::records(u16 target_id) const
	{
		const pair_state* p = find(target_id);
		return p ? &p->records : 0;
	}

	const slot_record* vision_sampler::sticky(u16 target_id) const
	{
		const pair_state* p = find(target_id);
		return p && p->sticky_set ? &p->sticky : 0;
	}

	void vision_sampler::retain_only(const xr_vector<u16>& target_ids)
	{
		for (u32 i = 0; i < m_pairs.size();)
		{
			if (std::find(target_ids.begin(), target_ids.end(), m_pairs[i].id) == target_ids.end())
			{
				m_pairs[i] = m_pairs.back();
				m_pairs.pop_back();
			}
			else
				++i;
		}
	}
}
