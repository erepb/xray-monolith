#include "stdafx.h"
#include "vision_feed.h"
#include "perception_config.h"
#include "occlusion_query.h"
#include "atmosphere.h"
#include "../entity_alive.h"
#include "../../xrEngine/xr_ioconsole.h"
#include "../../xrEngine/xr_ioc_cmd.h"

namespace perception
{
	static LPCSTR FEED_SECTION = "vision_feed";
	static LPCSTR SAMPLER_SECTION = "vision_sampler";

	static const feed_config FEED_DEFAULT = {1, 0.05f, 1.f, 1};

	feed_config vision_feed::s_config = FEED_DEFAULT;

	vision_feed::vision_feed(s32 seed) : m_sampler(seed)
	{
	}

	void vision_feed::on_level_load()
	{
		CInifile* ltx = open_config("vision feed");
		report_section(ltx, FEED_SECTION);
		report_section(ltx, SAMPLER_SECTION);

		feed_config c;
		c.enabled = cfg_flag(ltx, FEED_SECTION, "enabled", FEED_DEFAULT.enabled);
		c.visible_min = cfg_float(ltx, FEED_SECTION, "visible_min", FEED_DEFAULT.visible_min, 0.f, 1.f);
		c.rate_power = cfg_float(ltx, FEED_SECTION, "rate_power", FEED_DEFAULT.rate_power, 0.f, 4.f);
		c.points = cfg_u32(ltx, SAMPLER_SECTION, "points", FEED_DEFAULT.points, 1u, 64u);
		const float current_eps = cfg_float(ltx, SAMPLER_SECTION, "current_eps", CURRENT_EPS_DEFAULT, 0.f, 1.f);
		const u32 refresh_visits = cfg_u32(ltx, SAMPLER_SECTION, "refresh_visits", REFRESH_VISITS_DEFAULT, 1u, 64u);
		xr_delete(ltx);
		s_config = c;
		vision_sampler::configure(current_eps, refresh_visits);

		Msg("~ [perception] vision feed %s: visible_min %.2f, rate_power %.2f; sampler %u point(s) per vision tick, refresh within %.2f m, full every %u visit(s)",
		    c.enabled ? "on" : "off", c.visible_min, c.rate_power, c.points, current_eps, refresh_visits);
	}

	void vision_feed::register_console_commands()
	{
		CMD4(CCC_Integer, "perception_feed", &s_config.enabled, 0, 1);
		CMD4(CCC_Float, "perception_feed_visible_min", &s_config.visible_min, 0.f, 1.f);
		CMD4(CCC_Float, "perception_feed_rate_power", &s_config.rate_power, 0.f, 4.f);
	}

	bool vision_feed::fog_hides(const Fvector& eye, const Fvector& point)
	{
		const occlusion_query& q = occlusion_query::instance();
		segment seg;
		if (!q.ready() || !segment::make(eye, point, seg))
			return false;
		const float t = q.model().atmosphere_transmittance(atmosphere::instance().transmittance(seg));
		return point_fails(t, s_config.visible_min);
	}

	void vision_feed::drop_factor(u16 target_id)
	{
		for (u32 i = 0; i < m_factors.size(); ++i)
		{
			if (m_factors[i].id == target_id)
			{
				m_factors[i] = m_factors.back();
				m_factors.pop_back();
				return;
			}
		}
	}

	bool vision_feed::judge(CObject* observer, const Fvector& eye, const CObject* target, float threshold, u32 now,
	                        feed_verdict& out, trace_listener* listener)
	{
		if (!smart_cast<const CEntityAlive*>(target) || !target->CFORM())
			return false;

		trace_options limits;
		limits.clear_threshold = threshold;
		limits.visible_min = s_config.visible_min;
		m_sampler.sample(observer, eye, target, s_config.points, now, limits, 0, listener);

		pair_summary s;
		if (!m_sampler.summary(target, eye, threshold, s_config.visible_min, now, s) || !s.has_point)
		{
			drop_factor(target->ID());
			return false;
		}

		const float value = s_config.rate_power <= 0.f ? 1.f : (s.visibility > 0.f ? powf(s.visibility, s_config.rate_power) : 0.f);
		out.seen = s.seen;
		out.visibility = s.visibility;
		out.best = s.best;
		out.factor = value;
		out.bone = s.point_bone;
		out.local = s.point_local;
		out.current = s.current;
		out.limiting = s.limiting;
		out.degraded = s.degraded;

		for (u32 i = 0; i < m_factors.size(); ++i)
		{
			if (m_factors[i].id == target->ID())
			{
				m_factors[i].value = value;
				return true;
			}
		}
		factor f;
		f.id = target->ID();
		f.value = value;
		m_factors.push_back(f);
		return true;
	}

	float vision_feed::rate_factor(u16 target_id) const
	{
		for (u32 i = 0; i < m_factors.size(); ++i)
			if (m_factors[i].id == target_id)
				return m_factors[i].value;
		return 1.f;
	}

	void vision_feed::retain_only(const xr_vector<u16>& target_ids)
	{
		m_sampler.retain_only(target_ids);
		for (u32 i = 0; i < m_factors.size();)
		{
			if (std::find(target_ids.begin(), target_ids.end(), m_factors[i].id) == target_ids.end())
			{
				m_factors[i] = m_factors.back();
				m_factors.pop_back();
			}
			else
				++i;
		}
	}

	void vision_feed::clear()
	{
		m_sampler.clear();
		m_factors.clear();
	}
}
