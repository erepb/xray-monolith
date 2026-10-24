#include "stdafx.h"
#include "occluder_table.h"
#include "perception_debug.h"
#include "../../xrEngine/GameMtlLib.h"

namespace perception
{
	static occluder_class class_from_name(LPCSTR s)
	{
		if (0 == xr_strcmp(s, "hard")) return occ_hard;
		if (0 == xr_strcmp(s, "soft")) return occ_soft;
		if (0 == xr_strcmp(s, "creature")) return occ_creature;
		// "the canopy field already accounts for this": set on the invisible flora_collision proxy
		// materials, whose hits are dropped while a complete field exists so a bush is not counted twice.
		if (0 == xr_strcmp(s, "canopy")) return occ_canopy;
		return occ_class_count; // unrecognised
	}

	void occluder_table::on_level_unload()
	{
		xr_vector<u8>().swap(m_class);
	}

	void occluder_table::on_level_load()
	{
		const u32 n = GMLib.CountMaterial();
		m_class.assign(n, u8(occ_hard));

		for (u32 i = 0; i < n; ++i)
		{
			const SGameMtl* m = GMLib.GetMaterialByIdx(u16(i));
			if (!m)
				continue;
			// Default policy, per ENGINE-DESIGN 4.5: opaque materials are hard, anything the engine
			// already grades for vision is soft. On a clean install nothing is graded, so everything
			// lands on hard - that is a content gap, and it is meant to be visible in the report.
			if (m->fVisTransparencyFactor > 0.f)
				m_class[i] = u8(occ_soft);
		}

		apply_overrides();

#ifdef PERCEPTION_DEBUG
		u32 hist[occ_class_count];
		ZeroMemory(hist, sizeof(hist));
		for (u32 i = 0; i < n; ++i)
			hist[m_class[i]]++;
		Msg("~ [perception] occluder table: %u materials, hard %u, soft %u, creature %u, canopy %u",
		    n, hist[occ_hard], hist[occ_soft], hist[occ_creature], hist[occ_canopy]);
#endif
	}

	void occluder_table::apply_overrides()
	{
		// Same file and alias GameMtlLib.cpp:117 opens, so the `#include "materials_*.ltx"` chain is
		// inherited. The `@[` form is the SAFE override: it creates the base section when none exists
		// (Xr_ini.cpp:756-766, 889-900), which is what lets the two vanilla vegetation materials be
		// classified although they live only in gamemtl.xr. The strict `![` form would be dropped.
		string_path fn;
		if (!FS.exist(fn, _game_data_, "materials\\materials", ".ltx"))
			return;

		CInifile* ltx = xr_new<CInifile>(fn, TRUE);
		u32 applied = 0, rejected = 0;

		for (u32 i = 0; i < m_class.size(); ++i)
		{
			const SGameMtl* m = GMLib.GetMaterialByIdx(u16(i));
			if (!m || !ltx->section_exist(m->m_Name))
				continue;
			if (!ltx->line_exist(m->m_Name, "occluder_class"))
				continue;

			const occluder_class c = class_from_name(ltx->r_string(m->m_Name, "occluder_class"));
			if (c == occ_class_count)
			{
				Msg("! [perception] material [%s]: unknown occluder_class '%s'",
				    m->m_Name.c_str(), ltx->r_string(m->m_Name, "occluder_class"));
				++rejected;
				continue;
			}
			m_class[i] = u8(c);
			++applied;
		}

		xr_delete(ltx);

		if (applied || rejected)
			Msg("~ [perception] occluder_class overrides: %u applied, %u rejected", applied, rejected);
	}
}
