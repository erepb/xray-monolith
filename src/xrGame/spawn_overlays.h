////////////////////////////////////////////////////////////////////////////
//	Module 		: spawn_overlays.h
//	Created 	: 16.09.2026
//	Description : ltx overlays applied on top of the loaded all.spawn
////////////////////////////////////////////////////////////////////////////

#pragma once

#include "alife_spawn_registry.h"

class CInifile;

namespace spawn_overlays
{
	// Parses $game_config$\spawn_overlays.ltx (and its DLTX mods) once; apply_graph and apply_objects apply what
	// it found. One instance per CALifeSpawnRegistry::load.
	class CSpawnOverlays
	{
	private:
		CInifile* m_ini;
		float m_link_tolerance;
		bool m_sync_offsets;

	public:
		CSpawnOverlays();
		~CSpawnOverlays();

		// Syncs level offsets to game.ltx [levelNN] and applies [graph_links] / [graph_unlinks]. Returns a graph
		// over buffer (xr_malloc, caller owns) or nullptr when the base graph is unchanged.
		CGameGraph* apply_graph(const CGameGraph& base, void*& buffer);
		// Applies [spawn_remove] and [spawn_patch@<name>] to the spawn templates, then adds the records of
		// $game_spawn$\<level_name>\*.spawn fragments (SDK level.spawn format) as templates of that level.
		void apply_objects(CALifeSpawnRegistry::SPAWN_GRAPH& spawns, const CGameGraph& graph);
	};
}
