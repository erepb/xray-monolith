#pragma once

namespace perception
{
	static const char* const CFG_FILE = "creatures\\perception.ltx";

	// Null when the file is absent. The caller owns the result.
	inline CInifile* open_config(LPCSTR what)
	{
		string_path fn;
		if (!FS.exist(fn, "$game_config$", CFG_FILE))
		{
			Msg("! [perception] %s absent, %s at defaults", CFG_FILE, what);
			return 0;
		}
		return xr_new<CInifile>(fn, TRUE);
	}

	// The cfg_* readers return `def` silently for a null file or an absent section; the caller reports those.
	inline bool section_ready(CInifile* ltx, LPCSTR section)
	{
		return ltx && ltx->section_exist(section);
	}

	inline void report_section(CInifile* ltx, LPCSTR section)
	{
		if (ltx && !ltx->section_exist(section))
			Msg("! [perception] [%s] absent from %s, at defaults", section, CFG_FILE);
	}

	inline float cfg_float(CInifile* ltx, LPCSTR section, LPCSTR key, float def, float lo, float hi)
	{
		if (!section_ready(ltx, section))
			return def;
		if (!ltx->line_exist(section, key))
		{
			Msg("! [perception] [%s] %s missing from %s, using %.3f", section, key, CFG_FILE, def);
			return def;
		}
		const float v = ltx->r_float(section, key);
		const float c = _min(_max(v, lo), hi);
		if (c != v)
			Msg("! [perception] [%s] %s = %.3f out of range, using %.3f", section, key, v, c);
		return c;
	}

	inline u32 cfg_u32(CInifile* ltx, LPCSTR section, LPCSTR key, u32 def, u32 lo, u32 hi)
	{
		if (!section_ready(ltx, section))
			return def;
		if (!ltx->line_exist(section, key))
		{
			Msg("! [perception] [%s] %s missing from %s, using %u", section, key, CFG_FILE, def);
			return def;
		}
		const u32 v = ltx->r_u32(section, key);
		const u32 c = _min(_max(v, lo), hi);
		if (c != v)
			Msg("! [perception] [%s] %s = %u out of range, using %u", section, key, v, c);
		return c;
	}

	inline int cfg_flag(CInifile* ltx, LPCSTR section, LPCSTR key, int def)
	{
		if (!section_ready(ltx, section))
			return def;
		if (!ltx->line_exist(section, key))
		{
			Msg("! [perception] [%s] %s missing from %s, using %s", section, key, CFG_FILE, def ? "true" : "false");
			return def;
		}
		return ltx->r_bool(section, key) ? 1 : 0;
	}
}
