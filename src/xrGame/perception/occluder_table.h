#pragma once

#include "occlusion_types.h"

namespace perception
{
	// Material index -> occluder class, resolved once per level.
	//
	// Deliberately NOT a field on SGameMtl: that struct is deserialized from the binary gamemtl.xr
	// (GameMtlLib.cpp:49), so extending it would mean editing xrEngine for an xrGame feature. Reading
	// the same LTX chain here costs one INI parse and keeps the policy next to the code applying it.
	class occluder_table
	{
	public:
		void on_level_load();
		void on_level_unload();

		occluder_class of(u16 mtl_idx) const
		{
			return (mtl_idx < m_class.size()) ? occluder_class(m_class[mtl_idx]) : occ_hard;
		}

		u32 count() const { return m_class.size(); }

	private:
		void apply_overrides();

		xr_vector<u8> m_class;
	};
}
