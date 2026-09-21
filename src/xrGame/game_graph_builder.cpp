////////////////////////////////////////////////////////////////////////////
//	Module 		: game_graph_builder.cpp
//	Created 	: 16.09.2026
//	Description : Editable copy of a game graph that re-serializes into a spawn-chunk layout
////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "game_graph_builder.h"

CGameGraphBuilder::CGameGraphBuilder(const CGameGraph& base)
{
	m_header = base.header();
	m_cross_tables = base.m_cross_tables;
	m_edge_count = 0;
	m_dirty = false;

	const u32 vertex_count = base.header().vertex_count();
	m_vertices.resize(vertex_count);
	for (u32 i = 0; i < vertex_count; ++i)
	{
		const GameGraph::CVertex* source = base.vertex(i);
		SVertex& vertex = m_vertices[i];
		vertex.data = *source;

		CGameGraph::const_iterator e_begin, e_end;
		base.begin(i, e_begin, e_end);
		vertex.edges.reserve(e_end - e_begin);
		for (const auto* e = e_begin; e != e_end; ++e)
		{
			SEdge edge{};
			edge.vertex_id = e->vertex_id();
			edge.distance = e->distance();
			vertex.edges.push_back(edge);
		}
		m_edge_count += vertex.edges.size();

		CGameGraph::const_spawn_iterator p_begin, p_end;
		base.begin_spawn(i, p_begin, p_end);
		vertex.points.assign(p_begin, p_end);
	}
	VERIFY(m_edge_count == base.header().edge_count());
}

const GameGraph::SLevel* CGameGraphBuilder::level(LPCSTR level_name) const
{
	return m_header.level(level_name, true);
}

bool CGameGraphBuilder::find_vertex(GameGraph::_LEVEL_ID level_id, const Fvector& level_point, float tolerance, u32& vertex_id, float& distance) const
{
	float best = flt_max;
	vertex_id = u32(-1);
	for (u32 i = 0, n = m_vertices.size(); i < n; ++i)
	{
		const GameGraph::CVertex& vertex = m_vertices[i].data;
		if (vertex.level_id() != level_id)
			continue;
		const float current = vertex.level_point().distance_to(level_point);
		if (current >= best)
			continue;
		best = current;
		vertex_id = i;
	}
	distance = best;
	return vertex_id != u32(-1) && best <= tolerance;
}

bool CGameGraphBuilder::has_edge(u32 from, u32 to) const
{
	for (const SEdge& edge : m_vertices[from].edges)
		if (edge.vertex_id == to)
			return true;
	return false;
}

bool CGameGraphBuilder::add_edge(u32 from, u32 to)
{
	SVertex& vertex = m_vertices[from];
	if (vertex.edges.size() >= 255)
		return false;

	SEdge edge{};
	edge.vertex_id = static_cast<GameGraph::_GRAPH_ID>(to);
	edge.distance = vertex.data.game_point().distance_to(m_vertices[to].data.game_point());
	vertex.edges.push_back(edge);
	++m_edge_count;
	m_dirty = true;
	return true;
}

u32 CGameGraphBuilder::remove_edge(u32 from, u32 to)
{
	xr_vector<SEdge>& edges = m_vertices[from].edges;
	u32 removed = 0;
	for (u32 i = 0; i < edges.size();)
	{
		if (edges[i].vertex_id != to)
		{
			++i;
			continue;
		}
		edges.erase(edges.begin() + i);
		++removed;
	}
	m_edge_count -= removed;
	if (removed)
		m_dirty = true;
	return removed;
}

u32 CGameGraphBuilder::remove_level_edges(const GameGraph::_LEVEL_ID level_a, const GameGraph::_LEVEL_ID level_b)
{
	u32 removed = 0;
	for (SVertex& vertex : m_vertices)
	{
		const GameGraph::_LEVEL_ID from = vertex.data.level_id();
		if (from != level_a && from != level_b)
			continue;
		const GameGraph::_LEVEL_ID other = (from == level_a) ? level_b : level_a;
		xr_vector<SEdge>& edges = vertex.edges;
		for (u32 j = 0; j < edges.size();)
		{
			if (m_vertices[edges[j].vertex_id].data.level_id() != other)
			{
				++j;
				continue;
			}
			edges.erase(edges.begin() + j);
			++removed;
		}
	}
	m_edge_count -= removed;
	if (removed)
		m_dirty = true;
	return removed;
}

u32 CGameGraphBuilder::set_level_offset(const GameGraph::_LEVEL_ID level_id, const Fvector& offset)
{
	const auto level = m_header.m_levels.find(level_id);
	VERIFY(level != m_header.m_levels.end());

	Fvector delta;
	delta.sub(offset, level->second.m_offset);
	level->second.m_offset = offset;

	for (SVertex& vertex : m_vertices)
		if (vertex.data.level_id() == level_id)
			vertex.data.tGlobalPoint.add(delta);

	u32 reweighted = 0;
	for (SVertex& vertex : m_vertices)
	{
		const bool from_level = vertex.data.level_id() == level_id;
		for (u32 j = 0, m = vertex.edges.size(); j < m; ++j)
		{
			const SVertex& target = m_vertices[vertex.edges[j].vertex_id];
			if (from_level == (target.data.level_id() == level_id))
				continue;
			vertex.edges[j].distance = vertex.data.game_point().distance_to(target.data.game_point());
			++reweighted;
		}
	}

	m_dirty = true;
	return reweighted;
}

CGameGraph* CGameGraphBuilder::build(void*& buffer) const
{
	const u32 vertex_count = m_vertices.size();
	u32 point_count = 0;
	for (u32 i = 0; i < vertex_count; ++i)
		point_count += m_vertices[i].points.size();

	const u32 vertices_size = vertex_count * sizeof(GameGraph::CVertex);
	const u32 edges_size = m_edge_count * sizeof(GameGraph::CEdge);
	const u32 points_size = point_count * sizeof(GameGraph::CLevelPoint);

	buffer = xr_malloc(vertices_size + edges_size + points_size);
	auto* vertices = static_cast<GameGraph::CVertex*>(buffer);
	auto* edges = reinterpret_cast<GameGraph::CEdge*>(static_cast<u8*>(buffer) + vertices_size);
	auto* points = reinterpret_cast<GameGraph::CLevelPoint*>(static_cast<u8*>(buffer) + vertices_size + edges_size);

	u32 edge_offset = vertices_size;
	u32 point_offset = vertices_size + edges_size;
	for (u32 i = 0; i < vertex_count; ++i)
	{
		const SVertex& source = m_vertices[i];
		GameGraph::CVertex& vertex = vertices[i];
		vertex = source.data;
		vertex.dwEdgeOffset = edge_offset;
		vertex.dwPointOffset = point_offset;
		vertex.tNeighbourCount = u8(source.edges.size());
		vertex.tDeathPointCount = u8(source.points.size());

		for (u32 j = 0, n = source.edges.size(); j < n; ++j, ++edges)
		{
			edges->m_vertex_id = source.edges[j].vertex_id;
			edges->m_path_distance = source.edges[j].distance;
		}
		edge_offset += source.edges.size() * sizeof(GameGraph::CEdge);

		for (u32 j = 0, n = source.points.size(); j < n; ++j, ++points)
			*points = source.points[j];
		point_offset += source.points.size() * sizeof(GameGraph::CLevelPoint);
	}

	GameGraph::CHeader header = m_header;
	header.m_vertex_count = static_cast<GameGraph::_GRAPH_ID>(vertex_count);
	header.m_edge_count = m_edge_count;
	header.m_death_point_count = point_count;
	return xr_new<CGameGraph>(header, vertices, m_cross_tables);
}
