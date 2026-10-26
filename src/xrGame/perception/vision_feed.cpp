#include "stdafx.h"
#include "vision_feed.h"
#include "occlusion_query.h"
#include "../entity_alive.h"
#include "../../xrEngine/xr_ioconsole.h"
#include "../../xrEngine/xr_ioc_cmd.h"

namespace perception
{
	static LPCSTR CFG_FILE = "creatures\\perception.ltx";
	static LPCSTR FEED_SECTION = "vision_feed";
	static LPCSTR SAMPLER_SECTION = "vision_sampler";

	static const int ENABLED_DEFAULT = 1;
	static const float VISIBLE_MIN_DEFAULT = 0.05f;
	static const float RATE_POWER_DEFAULT = 1.f;
	static const u32 POINTS_DEFAULT = 2;
	static const float CURRENT_EPS_DEFAULT = 0.10f;
	static const u32 REFRESH_VISITS_DEFAULT = 4;

	feed_config vision_feed::s_config = {ENABLED_DEFAULT, VISIBLE_MIN_DEFAULT, RATE_POWER_DEFAULT, POINTS_DEFAULT};

	vision_feed::vision_feed(s32 seed) : m_sampler(seed)
	{
	}

	void vision_feed::on_level_load()
	{
		feed_config c = {ENABLED_DEFAULT, VISIBLE_MIN_DEFAULT, RATE_POWER_DEFAULT, POINTS_DEFAULT};
		float current_eps = CURRENT_EPS_DEFAULT;
		u32 refresh_visits = REFRESH_VISITS_DEFAULT;

		string_path fn;
		if (!FS.exist(fn, "$game_config$", CFG_FILE))
			Msg("! [perception] %s absent, vision feed at defaults", CFG_FILE);
		else
		{
			CInifile* ltx = xr_new<CInifile>(fn, TRUE);
			if (!ltx->section_exist(FEED_SECTION))
				Msg("! [perception] [%s] absent from %s, at defaults", FEED_SECTION, CFG_FILE);
			if (!ltx->section_exist(SAMPLER_SECTION))
				Msg("! [perception] [%s] absent from %s, at defaults", SAMPLER_SECTION, CFG_FILE);
			if (ltx->line_exist(FEED_SECTION, "enabled"))
				c.enabled = ltx->r_bool(FEED_SECTION, "enabled") ? 1 : 0;
			if (ltx->line_exist(FEED_SECTION, "visible_min"))
				c.visible_min = ltx->r_float(FEED_SECTION, "visible_min");
			if (ltx->line_exist(FEED_SECTION, "rate_power"))
				c.rate_power = ltx->r_float(FEED_SECTION, "rate_power");
			if (ltx->line_exist(SAMPLER_SECTION, "points"))
				c.points = ltx->r_u32(SAMPLER_SECTION, "points");
			if (ltx->line_exist(SAMPLER_SECTION, "current_eps"))
				current_eps = ltx->r_float(SAMPLER_SECTION, "current_eps");
			if (ltx->line_exist(SAMPLER_SECTION, "refresh_visits"))
				refresh_visits = ltx->r_u32(SAMPLER_SECTION, "refresh_visits");
			xr_delete(ltx);
		}

		clamp(c.visible_min, 0.f, 1.f);
		clamp(c.rate_power, 0.f, 4.f);
		clamp(c.points, 1u, 64u);
		clamp(current_eps, 0.f, 1.f);
		clamp(refresh_visits, 1u, 64u);
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
			return false;

		const float value = s.visibility > 0.f ? powf(s.visibility, s_config.rate_power) : 0.f;
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
