#include "stdafx.h"
#include "vision_sampler.h"
#include "occlusion_query.h"
#include "bone_shape.h"
#include "../../Include/xrRender/Kinematics.h"
#include "../../xrEngine/bone.h"

namespace perception
{
	vision_sampler::vision_sampler(s32 seed) : m_rnd(seed)
	{
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

	void vision_sampler::sample(CObject* observer, const Fvector& eye, const CObject* target, u32 k, u32 now,
	                            trace_listener* listener)
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

		u32 traced = 0;
		for (u32 tried = 0; tried < n && traced < k; ++tried)
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

			const Fvector local = random_point_on_bone_shape(K->LL_GetData(s.bone).shape, m_rnd);
			const Fvector world = target->get_last_local_point_on_mesh(local, s.bone);

			segment seg;
			if (!segment::make(eye, world, seg))
				continue;

			occlusion_result r;
			if (!q.trace(seg, CDB::OPT_CULL, observer, target, r))
				continue;

			rec.eye = eye;
			rec.point = world;
			for (u32 c = 0; c < occ_class_count; ++c)
				rec.channel[c] = r.channel[c];
			rec.t = r.transmittance;
			rec.t_v1 = r.v1_transmittance();
			rec.dominant = r.dominant;
			rec.degraded = r.degraded;
			rec.self_occluded = r.self_occluded(seg.length);
			rec.time = now;
			++rec.visits;
			++traced;

			if (listener)
				listener->on_trace(seg, s.bone, r);
		}
	}

	bool vision_sampler::summary(const CObject* target, float clear_threshold, u32 now, pair_summary& out) const
	{
		ZeroMemory(&out, sizeof(out));
		out.limiting = occluder_class(occ_class_count);

		const pair_state* p = target ? find(target->ID()) : 0;
		if (!p || p->object != target || p->slots.empty())
			return false;

		IKinematics* K = smart_cast<IKinematics*>(target->Visual());
		if (!K)
			return false;

		float w = 0.f, clear = 0.f, clear_v1 = 0.f, clear_v1_self = 0.f, self = 0.f, t = 0.f;
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
		}

		if (!out.sampled || w <= 0.f)
			return false;

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
		return true;
	}

	const xr_vector<slot_record>* vision_sampler::records(u16 target_id) const
	{
		const pair_state* p = find(target_id);
		return p ? &p->records : 0;
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
