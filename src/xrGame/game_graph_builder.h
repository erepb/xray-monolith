////////////////////////////////////////////////////////////////////////////
//	Module 		: game_graph_builder.h
//	Created 	: 16.09.2026
//	Description : Editable copy of a game graph that re-serializes into a spawn-chunk layout
////////////////////////////////////////////////////////////////////////////

#pragma once

#include "game_graph.h"

class CGameGraphBuilder
{
public:
	struct SEdge
	{
		GameGraph::_GRAPH_ID vertex_id;
		float distance;
	};

	struct SVertex
	{
		GameGraph::CVertex data;
		xr_vector<SEdge> edges;
		xr_vector<GameGraph::CLevelPoint> points;
	};

private:
	GameGraph::CHeader m_header;
	xr_vector<SVertex> m_vertices;
	u32* m_cross_tables;
	u32 m_edge_count;
	bool m_dirty;

public:
	explicit CGameGraphBuilder(const CGameGraph& base);
	IC const GameGraph::CHeader& header() const { return m_header; }
	IC u32 vertex_count() const { return m_vertices.size(); }
	IC const SVertex& vertex(u32 vertex_id) const { return m_vertices[vertex_id]; }
	IC bool dirty() const { return m_dirty; }

	const GameGraph::SLevel* level(LPCSTR level_name) const;
	bool find_vertex(GameGraph::_LEVEL_ID level_id, const Fvector& level_point, float tolerance, u32& vertex_id, float& distance) const;
	bool has_edge(u32 from, u32 to) const;
	bool add_edge(u32 from, u32 to);
	u32 remove_edge(u32 from, u32 to);
	u32 remove_level_edges(GameGraph::_LEVEL_ID level_a, GameGraph::_LEVEL_ID level_b);
	u32 set_level_offset(GameGraph::_LEVEL_ID level_id, const Fvector& offset);

	// Lays vertices, edges and death points out into buffer (xr_malloc, caller owns) and
	// returns a graph over it that keeps using the base cross tables.
	CGameGraph* build(void*& buffer) const;
};
