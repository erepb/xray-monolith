////////////////////////////////////////////////////////////////////////////
//	Module 		: patrol_path_storage.h
//	Created 	: 15.06.2004
//  Modified 	: 15.06.2004
//	Author		: Dmitriy Iassenev
//	Description : Patrol path storage
////////////////////////////////////////////////////////////////////////////

#pragma once

class CPatrolPath;
class CLevelGraph;
class CGameLevelCrossTable;
class CGameGraph;

#include "object_interfaces.h"
#include "associative_vector.h"

class CPatrolPathStorage : public IPureSerializeObject<IReader, IWriter>
{
private:
	typedef IPureSerializeObject<IReader, IWriter> inherited;

public:
	typedef associative_vector<shared_str, CPatrolPath*> PATROL_REGISTRY;
	typedef PATROL_REGISTRY::iterator iterator;
	typedef PATROL_REGISTRY::const_iterator const_iterator;

protected:
	PATROL_REGISTRY m_registry;

public:
	IC CPatrolPathStorage();
	virtual ~CPatrolPathStorage();
	virtual void load(IReader& stream); 
	// game_graph = nullptr (no ALifeSimulator): 'level =' sections are skipped
	virtual void load_from_config(const CGameGraph* game_graph);
	virtual void save(IWriter& stream);

public:
	void load_raw(const CLevelGraph* level_graph, const CGameLevelCrossTable* cross, const CGameGraph* game_graph, IReader& stream);
	// Snaps the config paths of the loaded level that were approximated at load
	void resolve_level(const CLevelGraph* level_graph, const CGameLevelCrossTable* cross, const CGameGraph* game_graph);
	IC const CPatrolPath* path(shared_str patrol_name, bool no_assert = false) const;
	IC const PATROL_REGISTRY& patrol_paths() const;
};

#include "patrol_path_storage_inline.h"
