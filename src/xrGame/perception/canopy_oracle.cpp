#include "stdafx.h"
#include <algorithm>
#include "canopy_oracle.h"
#include "canopy_data.h"
#include "slot_walk.h"
#include "perception_debug.h"

namespace perception
{
	// Columns per axis of the crown index.
	static const u32 MAX_COLUMNS_PER_AXIS = 4096;

	// Largest world coordinate a crown box may reach. Well inside what iFloor can convert, so the
	// column arithmetic below cannot overflow int.
	static const float MAX_WORLD_COORD = 1.0e6f;

	static const u32 BVH_LEAF = 4;

	// A median split halves the triangle count at every level, so depth stays under 33 and the stack
	// never holds more than depth + 1 entries.
	static const u32 BVH_STACK = 64;

	// Crown boxes are grown by this before indexing and clipping, so a ray entering a box exactly on a
	// column boundary still finds the crown listed in the column it is walking.
	static const float BOX_PAD = 0.01f;

	// A footprint fully covered by leaves blocks completely while its optical depth stays finite.
	static const float COVERAGE_MAX = 0.999f;

	// log2 from the exponent and a linear mantissa: within 0.09 of a level, and no log call per hit.
	static IC float fast_log2(float x)
	{
		u32 b;
		CopyMemory(&b, &x, 4);
		const float e = float(int((b >> 23) & 0xff) - 127);
		b = (b & 0x007fffffu) | 0x3f800000u;
		float m;
		CopyMemory(&m, &b, 4);
		return e + (m - 1.f);
	}

	static IC float slab_inv(float d)
	{
		return _abs(d) > 1e-12f ? 1.f / d : (d >= 0.f ? 1e30f : -1e30f);
	}

	// Clips [t0, t1] of o + d * t to the box; inv is 1 / d per axis.
	static IC bool clip_box(const Fvector& o, const Fvector& inv, const Fvector& bmin, const Fvector& bmax,
	                        float& t0, float& t1)
	{
		for (u32 k = 0; k < 3; ++k)
		{
			float ta = ((&bmin.x)[k] - (&o.x)[k]) * (&inv.x)[k];
			float tb = ((&bmax.x)[k] - (&o.x)[k]) * (&inv.x)[k];
			if (ta > tb)
				std::swap(ta, tb);
			t0 = _max(t0, ta);
			t1 = _min(t1, tb);
			if (t0 > t1)
				return false;
		}
		return true;
	}

	// Two-sided: cards are seen from both faces, and an instance matrix may mirror. On a hit, t is the
	// distance along d and (u, w) the barycentrics of v[1] and v[2].
	static IC bool pierces(const Fvector& o, const Fvector& d, const Fvector* v, float t0, float t1,
	                       float& t, float& u, float& w)
	{
		Fvector e1, e2, p, tv, q;
		e1.sub(v[1], v[0]);
		e2.sub(v[2], v[0]);
		p.crossproduct(d, e2);
		const float det = e1.dotproduct(p);
		if (_abs(det) < 1e-12f)
			return false;
		const float inv = 1.f / det;
		tv.sub(o, v[0]);
		u = tv.dotproduct(p) * inv;
		if (u < 0.f || u > 1.f)
			return false;
		q.crossproduct(tv, e1);
		w = d.dotproduct(q) * inv;
		if (w < 0.f || u + w > 1.f)
			return false;
		t = e2.dotproduct(q) * inv;
		return t > t0 && t < t1;
	}

	canopy_oracle& canopy_oracle::instance()
	{
		static canopy_oracle s_oracle;
		return s_oracle;
	}

	void canopy_oracle::on_level_unload()
	{
		xr_vector<Fvector>().swap(m_tris);
		xr_vector<Fvector2>().swap(m_uvs);
		xr_vector<float>().swap(m_tri_lod);
		xr_vector<coverage_lod>().swap(m_coverage);
		xr_vector<bvh_node>().swap(m_nodes);
		xr_vector<crown_model>().swap(m_models);
		xr_vector<crown>().swap(m_crowns);
		xr_vector<u32>().swap(m_col);
		xr_vector<u32>().swap(m_ids);
		m_min_sx = m_min_sz = 0;
		m_size_x = m_size_z = 0;
		m_y_lo = m_y_hi = 0.f;
		m_ready = false;
		m_complete = false;
		m_overcounted = false;
		m_march_id = 0;
		PDBG_ONLY(ZeroMemory(&m_stats, sizeof(m_stats)));
	}

	void canopy_oracle::on_level_load()
	{
		on_level_unload();

		CTimer t;
		t.Start();
		const bool ok = build();
		PDBG_ONLY(m_stats.bake_ms = t.GetElapsed_sec() * 1000.f);

		if (!ok)
		{
			on_level_unload();
			Msg("~ [perception] canopy: no crowns - trees will not occlude");
			return;
		}

		m_ready = true;

#ifdef PERCEPTION_DEBUG
		size_t coverage = 0;
		m_stats.res_min = u32(-1);
		m_stats.res_max = 0;
		for (u32 i = 0; i < m_coverage.size(); ++i)
		{
			coverage += m_coverage[i].bytes();
			m_stats.res_min = _min(m_stats.res_min, m_coverage[i].res());
			m_stats.res_max = _max(m_stats.res_max, m_coverage[i].res());
		}
		m_stats.textures = u32(m_coverage.size());
		m_stats.coverage_mb = float(coverage) / (1024.f * 1024.f);
		m_stats.memory_mb = float(m_tris.size() * sizeof(Fvector) + m_uvs.size() * sizeof(Fvector2)
		                          + m_tri_lod.size() * sizeof(float) + coverage + m_nodes.size() * sizeof(bvh_node)
		                          + m_models.size() * sizeof(crown_model) + m_crowns.size() * sizeof(crown)
		                          + (m_col.size() + m_ids.size()) * sizeof(u32)) / (1024.f * 1024.f);
		Msg("~ [perception] canopy: %u crown(s) over %u model(s), %u triangle(s) (%u twin(s) dropped), %u BVH node(s), %ux%u columns offs (%d,%d), %u index entries  %.2f MB  bake %.1f ms  %s",
		    u32(m_crowns.size()), u32(m_models.size()), u32(m_tris.size() / 3), m_stats.twins, u32(m_nodes.size()),
		    m_size_x, m_size_z, m_min_sx, m_min_sz, u32(m_ids.size()), m_stats.memory_mb, m_stats.bake_ms,
		    m_complete ? "complete" : "INCOMPLETE");
		Msg("~ [perception] canopy: %u coverage pyramid(s), base %u..%u, %.2f MB, finest cell %.3f m; geometry and textures read in %.1f ms",
		    m_stats.textures, m_stats.textures ? m_stats.res_min : 0, m_stats.res_max, m_stats.coverage_mb,
		    m_finest_cell, m_stats.data_ms);
#endif
	}

	u32 canopy_oracle::build_node(const canopy_model& cm, const xr_vector<float>& tri_lod,
	                              const xr_vector<Fvector>& centroid, xr_vector<u32>& ids, u32 lo, u32 hi)
	{
		const xr_vector<Fvector>& verts = cm.verts;
		const u32 self = u32(m_nodes.size());
		m_nodes.push_back(bvh_node());

		Fbox b;
		b.invalidate();
		for (u32 k = lo; k < hi; ++k)
			for (u32 j = 0; j < 3; ++j)
				b.modify(verts[ids[k] * 3 + j]);
		m_nodes[self].bmin = b.min;
		m_nodes[self].bmax = b.max;

		if (hi - lo <= BVH_LEAF)
		{
			m_nodes[self].a = u32(m_tris.size() / 3);
			m_nodes[self].count = hi - lo;
			for (u32 k = lo; k < hi; ++k)
			{
				for (u32 j = 0; j < 3; ++j)
				{
					m_tris.push_back(verts[ids[k] * 3 + j]);
					m_uvs.push_back(cm.uvs[ids[k] * 3 + j]);
				}
				m_tri_lod.push_back(tri_lod[ids[k]]);
			}
			return self;
		}

		Fbox cb;
		cb.invalidate();
		for (u32 k = lo; k < hi; ++k)
			cb.modify(centroid[ids[k]]);
		Fvector ext;
		ext.sub(cb.max, cb.min);
		const u32 axis = (ext.x >= ext.y && ext.x >= ext.z) ? 0 : (ext.y >= ext.z ? 1 : 2);

		const u32 mid = (lo + hi) / 2;
		std::nth_element(ids.begin() + lo, ids.begin() + mid, ids.begin() + hi,
		                 [&](u32 l, u32 r) { return (&centroid[l].x)[axis] < (&centroid[r].x)[axis]; });

		build_node(cm, tri_lod, centroid, ids, lo, mid);
		const u32 right = build_node(cm, tri_lod, centroid, ids, mid, hi);
		m_nodes[self].a = right;
		m_nodes[self].count = 0;
		return self;
	}

	bool canopy_oracle::build()
	{
		canopy_data cd;
		CTimer t;
		t.Start();
		const bool loaded = cd.load(m_finest_cell, m_alpha_ref);
		PDBG_ONLY(m_stats.data_ms = t.GetElapsed_sec() * 1000.f);
		if (!loaded)
			return false;
		m_coverage.swap(cd.coverage());
		PDBG_ONLY(m_stats.twins = cd.twins());

		const xr_vector<canopy_model>& models = cd.models();
		const xr_vector<canopy_instance>& insts = cd.instances();

		if (cd.non_tree_leaf())
			Msg("! [perception] canopy: %u leaf-shader visual(s) are not a tree type and were skipped - "
			    "canopy coverage on this level is incomplete", cd.non_tree_leaf());
		if (cd.unresolved_swi())
			Msg("! [perception] canopy: %u progressive-mesh crown(s) with no sliding window - built at "
				"full index range, leaf area over-counted", cd.unresolved_swi());
		if (cd.no_texture())
			Msg("! [perception] canopy: %u model(s) with no texture - contribute nothing",
				cd.no_texture());

		// unresolved_swi is deliberately not part of this: such a crown is built at the full index
		// range, which over-counts its leaf area rather than omitting it. Over-counting is the safe
		// direction and never a false-clear, whereas clearing m_complete would disable proxy deferral
		// for every bush on the level and trade a harmless over-count for real double-counting.
		m_complete = cd.non_tree_leaf() == 0 && cd.rejected() == 0 && cd.no_texture() == 0
			&& cd.skipped() == 0;
		m_overcounted = cd.unresolved_swi() != 0;

		if (cd.skipped())
			Msg("! [perception] canopy: %u skip event(s) left leaf geometry unusable - the crowns do not account for them",
				cd.skipped());
		if (cd.rejected())
			Msg("! [perception] canopy: %u texture(s) unreadable - those crowns contribute nothing",
			    cd.rejected());

		// One BVH per model with leaves. Models without (frond stubs, coverage 0) get none, and their
		// crowns are not placed.
		xr_vector<u32> model_slot(models.size(), u32(-1));
		{
			xr_vector<u32> ids;
			xr_vector<Fvector> centroid;
			xr_vector<float> tri_lod;
			for (u32 m = 0; m < models.size(); ++m)
			{
				const canopy_model& cm = models[m];
				const u32 n = u32(cm.verts.size() / 3);
				if (!n || cm.coverage >= m_coverage.size() || m_coverage[cm.coverage].mean() <= 0.f)
					continue;

				const float res = float(m_coverage[cm.coverage].res());
				ids.resize(n);
				centroid.resize(n);
				tri_lod.resize(n);
				for (u32 i = 0; i < n; ++i)
				{
					const Fvector* v = &cm.verts[i * 3];
					const Fvector2* uv = &cm.uvs[i * 3];
					ids[i] = i;
					centroid[i].add(v[0], v[1]).add(v[2]).div(3.f);

					// Base texels per object metre on this card: res / sqrt(area / UV area).
					Fvector e1, e2, c;
					e1.sub(v[1], v[0]);
					e2.sub(v[2], v[0]);
					const float area = c.crossproduct(e1, e2).magnitude();
					const float uv_area = _abs((uv[1].x - uv[0].x) * (uv[2].y - uv[0].y)
					                           - (uv[1].y - uv[0].y) * (uv[2].x - uv[0].x));
					tri_lod[i] = (area > 1e-8f && uv_area > 1e-12f) ? log2f(res / _sqrt(area / uv_area)) : 0.f;
				}

				crown_model md;
				md.coverage = cm.coverage;
				md.root = build_node(cm, tri_lod, centroid, ids, 0, n);
				model_slot[m] = u32(m_models.size());
				m_models.push_back(md);
			}
		}
		if (m_models.empty())
			return false;

		u32 singular = 0;
		Fbox world;
		world.invalidate();
		m_crowns.reserve(insts.size());
		for (u32 i = 0; i < insts.size(); ++i)
		{
			const canopy_instance& inst = insts[i];
			if (inst.model >= models.size() || u32(-1) == model_slot[inst.model])
				continue;

			crown c;
			if (!c.inv.invert_b(inst.xform))
			{
				++singular;
				continue;
			}
			c.model = model_slot[inst.model];
			c.stamp = 0;
			const Fmatrix& x = inst.xform;
			const float det = x._11 * (x._22 * x._33 - x._23 * x._32) - x._12 * (x._21 * x._33 - x._23 * x._31)
				+ x._13 * (x._21 * x._32 - x._22 * x._31);
			c.lod = -log2f(_abs(det)) / 3.f;

			const bvh_node& root = m_nodes[m_models[c.model].root];
			Fbox local;
			local.set(root.bmin, root.bmax);
			c.box.xform(local, inst.xform);
			c.box.grow(BOX_PAD);
			if (!_valid(c.box.min) || !_valid(c.box.max)
				|| _abs(c.box.min.x) > MAX_WORLD_COORD || _abs(c.box.max.x) > MAX_WORLD_COORD
				|| _abs(c.box.min.y) > MAX_WORLD_COORD || _abs(c.box.max.y) > MAX_WORLD_COORD
				|| _abs(c.box.min.z) > MAX_WORLD_COORD || _abs(c.box.max.z) > MAX_WORLD_COORD)
			{
				++singular;
				continue;
			}

			world.merge(c.box);
			m_crowns.push_back(c);
		}
		if (singular)
		{
			m_complete = false;
			Msg("! [perception] canopy: %u crown(s) with a singular or out-of-range transform were skipped",
			    singular);
		}
		if (m_crowns.empty())
			return false;

		m_min_sx = iFloor(world.min.x / SLOT_SIZE);
		m_min_sz = iFloor(world.min.z / SLOT_SIZE);
		const u32 nx = u32(iFloor(world.max.x / SLOT_SIZE) - m_min_sx) + 1;
		const u32 nz = u32(iFloor(world.max.z / SLOT_SIZE) - m_min_sz) + 1;
		if (nx > MAX_COLUMNS_PER_AXIS || nz > MAX_COLUMNS_PER_AXIS)
		{
			Msg("! [perception] canopy: extent %ux%u columns exceeds the index layout - no crowns", nx, nz);
			return false;
		}
		m_size_x = nx;
		m_size_z = nz;
		m_y_lo = world.min.y;
		m_y_hi = world.max.y;

		// Two passes over the crown footprints: count per column, then fill.
		xr_vector<u32> count(nx * nz, 0);
		for (u32 pass = 0; pass < 2; ++pass)
		{
			if (1 == pass)
			{
				u32 total = 0;
				m_col.assign(nx * nz, 0);
				for (u32 k = 0; k < count.size(); ++k)
				{
					const u32 n = _min(count[k], COL_COUNT_MAX);
					m_col[k] = (total << COL_COUNT_BITS) | n;
					total += n;
				}
				if (total > COL_OFFSET_MAX)
				{
					Msg("! [perception] canopy: %u index entries exceed the %u the directory can address - no crowns",
					    total, COL_OFFSET_MAX);
					return false;
				}
				m_ids.assign(total, 0);
			}

			xr_vector<u32> filled;
			if (1 == pass)
				filled.assign(nx * nz, 0);

			for (u32 i = 0; i < m_crowns.size(); ++i)
			{
				const Fbox& b = m_crowns[i].box;
				const int x0 = iFloor(b.min.x / SLOT_SIZE) - m_min_sx, x1 = iFloor(b.max.x / SLOT_SIZE) - m_min_sx;
				const int z0 = iFloor(b.min.z / SLOT_SIZE) - m_min_sz, z1 = iFloor(b.max.z / SLOT_SIZE) - m_min_sz;
				for (int gz = z0; gz <= z1; ++gz)
					for (int gx = x0; gx <= x1; ++gx)
					{
						const u32 k = u32(gz) * nx + u32(gx);
						if (0 == pass)
							++count[k];
						else if (filled[k] < (m_col[k] & COL_COUNT_MAX))
							m_ids[(m_col[k] >> COL_COUNT_BITS) + filled[k]++] = i;
					}
			}
		}

		u32 dropped = 0;
		for (u32 k = 0; k < count.size(); ++k)
			dropped += count[k] - (m_col[k] & COL_COUNT_MAX);
		if (dropped)
		{
			m_complete = false;
			Msg("! [perception] canopy: %u crown listing(s) past the %u-per-column limit were dropped", dropped,
			    COL_COUNT_MAX);
		}

#ifdef PERCEPTION_DEBUG
		m_stats.instances = u32(m_crowns.size());
		m_stats.models = u32(m_models.size());
		m_stats.triangles = u32(m_tris.size() / 3);
		m_stats.nodes = u32(m_nodes.size());
#endif
		return true;
	}

	bool canopy_oracle::band_at(float x, float z, float& y_lo, float& y_hi) const
	{
		if (!m_ready)
			return false;

		const int gx = iFloor(x / SLOT_SIZE) - m_min_sx;
		const int gz = iFloor(z / SLOT_SIZE) - m_min_sz;
		if (gx < 0 || gz < 0 || u32(gx) >= m_size_x || u32(gz) >= m_size_z)
			return false;

		const u32 c = column(gx, gz);
		const u32 count = c & COL_COUNT_MAX;
		if (!count)
			return false;

		const u32 first = c >> COL_COUNT_BITS;
		y_lo = flt_max;
		y_hi = -flt_max;
		for (u32 k = 0; k < count; ++k)
		{
			const Fbox& b = m_crowns[m_ids[first + k]].box;
			y_lo = _min(y_lo, b.min.y);
			y_hi = _max(y_hi, b.max.y);
		}
		return true;
	}

	float canopy_oracle::pierce(const crown& c, const Fvector& from, const Fvector& dir, float t0, float t1,
	                            const cone_along& cone, u32& hits, xr_vector<canopy_hit>* rec) const
	{
		// The inverse is affine, so the object-space direction stays unnormalised and t keeps meaning
		// world metres.
		Fvector o, d, inv;
		c.inv.transform_tiny(o, from);
		c.inv.transform_dir(d, dir);
		inv.set(slab_inv(d.x), slab_inv(d.y), slab_inv(d.z));

		const crown_model& md = m_models[c.model];
		const coverage_lod& cov = m_coverage[md.coverage];

		u32 stack[BVH_STACK];
		u32 sp = 0;
		stack[sp++] = md.root;

		float tau = 0.f;
		hits = 0;
		while (sp)
		{
			const u32 ni = stack[--sp];
			const bvh_node& n = m_nodes[ni];
			PDBG_ONLY(++m_stats.node_tests);

			float a = t0, b = t1;
			if (!clip_box(o, inv, n.bmin, n.bmax, a, b))
				continue;

			if (n.count)
			{
				PDBG_ONLY(m_stats.triangle_tests += n.count);
				for (u32 k = 0; k < n.count; ++k)
				{
					const u32 tri = n.a + k;
					float t, bu, bw;
					if (!pierces(o, d, &m_tris[tri * 3], t0, t1, t, bu, bw))
						continue;

					canopy_hit h;
					h.crown = u32(&c - &m_crowns[0]);
					h.t = t;
					h.footprint = cone.eye + cone.slope * t;
					h.near_plane = t < cone.near_plane;
					h.level = 0.f;
					h.coverage = 0.f;
					if (!h.near_plane)
					{
						const Fvector2* uv = &m_uvs[tri * 3];
						const float bv = 1.f - bu - bw;
						const float u = uv[0].x * bv + uv[1].x * bu + uv[2].x * bw;
						const float v = uv[0].y * bv + uv[1].y * bu + uv[2].y * bw;
						h.level = fast_log2(h.footprint) + m_tri_lod[tri] + c.lod;
						h.coverage = _min(cov.sample(u, v, h.level), COVERAGE_MAX);
						if (h.coverage > 0.f)
							tau -= logf(1.f - h.coverage);
						++hits;
					}
					if (rec)
						rec->push_back(h);
				}
				continue;
			}

			stack[sp++] = n.a;
			stack[sp++] = ni + 1;
		}
		return tau;
	}

	void canopy_oracle::march(const Fvector& from, const Fvector& to, const sight_cone& cone, march_result& out,
	                          xr_vector<veg_chord>* chords, xr_vector<canopy_hit>* hit_list) const
	{
		out.tau = 0.f;
		out.crowns = 0;
		if (!m_ready)
			return;

		PDBG_ONLY(++m_stats.marches);

		// Whole-index reject, a conservative box test. Without it a ray that never comes near a crown
		// still pays the full column walk to fail every bounds check.
		if (_max(from.y, to.y) < m_y_lo || _min(from.y, to.y) > m_y_hi)
			return;
		const float fx0 = float(m_min_sx) * SLOT_SIZE;
		const float fx1 = float(m_min_sx + int(m_size_x)) * SLOT_SIZE;
		const float fz0 = float(m_min_sz) * SLOT_SIZE;
		const float fz1 = float(m_min_sz + int(m_size_z)) * SLOT_SIZE;
		if (_max(from.x, to.x) < fx0 || _min(from.x, to.x) > fx1
			|| _max(from.z, to.z) < fz0 || _min(from.z, to.z) > fz1)
			return;

		slot_walk w(from, to);
		if (!w.valid())
			return;
		const Fvector& dir = w.dir();
		const float len = w.len();

		Fvector inv;
		inv.set(slab_inv(dir.x), slab_inv(dir.y), slab_inv(dir.z));

		cone_along ca;
		ca.eye = _max(cone.eye, 1e-3f);
		ca.slope = (_max(cone.target, ca.eye) - ca.eye) / len;
		ca.near_plane = cone.near_plane;

		const u32 id = ++m_march_id;

		while (w.next())
		{
			const int gx = w.sx() - m_min_sx;
			const int gz = w.sz() - m_min_sz;
			if (gx < 0 || gz < 0 || u32(gx) >= m_size_x || u32(gz) >= m_size_z)
				continue;

			const u32 c = column(gx, gz);
			const u32 count = c & COL_COUNT_MAX;
			if (!count)
				continue;

			PDBG_ONLY(++m_stats.columns_walked);

			const u32 first = c >> COL_COUNT_BITS;
			for (u32 k = 0; k < count; ++k)
			{
				const crown& cr = m_crowns[m_ids[first + k]];
				if (cr.stamp == id)
					continue;
				cr.stamp = id;
				PDBG_ONLY(++m_stats.boxes_tested);

				float t0 = 0.f, t1 = len;
				if (!clip_box(from, inv, cr.box.min, cr.box.max, t0, t1))
					continue;
				PDBG_ONLY(++m_stats.boxes_entered);

				u32 hits = 0;
				const float tau = pierce(cr, from, dir, t0, t1, ca, hits, hit_list);
				if (hits)
				{
					out.tau += tau;
					++out.crowns;
					PDBG_ONLY(m_stats.hits += hits);
				}

				// A crossed crown with no card hit is reported too: the debug view shows a ray that went
				// between the cards rather than nowhere near them.
				if (chords)
				{
					veg_chord ch;
					ch.enter.mad(from, dir, t0);
					ch.exit.mad(from, dir, t1);
					ch.blocked = 1.f - expf(-tau);
					ch.tau = tau;
					ch.origin.set((cr.box.min.x + cr.box.max.x) * .5f, cr.box.min.y, (cr.box.min.z + cr.box.max.z) * .5f);
					ch.radius = _max(cr.box.max.x - cr.box.min.x, cr.box.max.z - cr.box.min.z) * .5f;
					ch.y_lo = cr.box.min.y;
					ch.y_hi = cr.box.max.y;
					ch.model = u16(cr.model);
					ch.crown = u32(&cr - &m_crowns[0]);
					ch.hits = u16(_min(hits, 0xffffu));
					ch.inside = !!cr.box.contains(from);
					chords->push_back(ch);
				}
			}
		}
	}
}
