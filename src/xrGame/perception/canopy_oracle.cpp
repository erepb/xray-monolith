#include "stdafx.h"
#include <algorithm>
#include "canopy_oracle.h"
#include "canopy_data.h"
#include "slot_walk.h"
#include "perception_debug.h"

namespace perception
{
	static const float CELL_VOLUME = SLOT_SIZE * SLOT_SIZE * SLOT_SIZE;

	// Columns per axis. 12 bits each in the bake key, and well past the 377x736 Escape needs.
	static const u32 MAX_COLUMNS_PER_AXIS = 4096;

	// Row stride of the bake key (column * MAX_ROWS + row); the row index is 8 bits in both the key and
	// the stored run. Only MAX_ROWS - 1 rows are usable: at the corner of a maximal 4096 x 4096 field
	// row 255 would make the key 0xFFFFFFFF, which is bake_map::EMPTY, and that cell would be lost
	// silently. 255 rows is 510 m of canopy against the 127 m Escape spans.
	static const u32 MAX_ROWS = 256;

	// Largest world coordinate the extent is allowed to start from. Well inside what iFloor can
	// convert, so the slot-index arithmetic below cannot overflow int.
	static const float MAX_WORLD_COORD = 1.0e6f;

	// A cell holding less than this contributes under 1e-3 nepers per metre of travel and is not
	// worth a run. Keeps slivers clipped by a cell boundary out of the field.
	static const float MIN_CELL_EXT = 1e-3f;

	namespace
	{
		// Open-addressed accumulator for the bake. A red-black tree would cost 3.37 M node inserts and
		// a (key, value) array would cost 67 MB; this holds the ~223 k distinct cells in single-digit
		// megabytes with O(1) inserts.
		class bake_map
		{
		public:
			static const u32 EMPTY = 0xFFFFFFFF;

			bake_map() : m_mask(0), m_used(0), m_shift(32) {}

			void init(u32 slots)
			{
				u32 n = 1024;
				while (n < slots)
					n <<= 1;
				m_key.assign(n, EMPTY);
				m_val.assign(n, 0.f);
				m_mask = n - 1;
				m_used = 0;
				m_shift = 32;
				for (u32 s = n; s > 1; s >>= 1)
					--m_shift;
			}

			void add(u32 k, float v)
			{
				// The mask is belt-and-braces: the shifted hash is already below the slot count.
				u32 i = hash(k) & m_mask;
				while (EMPTY != m_key[i] && m_key[i] != k)
					i = (i + 1) & m_mask;
				if (EMPTY == m_key[i])
				{
					m_key[i] = k;
					++m_used;
					if (m_used * 10 > (m_mask + 1) * 7)
					{
						m_val[i] = v;
						grow();
						return;
					}
				}
				m_val[i] += v;
			}

			u32 used() const { return m_used; }
			u32 slots() const { return m_mask + 1; }
			u32 key_at(u32 i) const { return m_key[i]; }
			float val_at(u32 i) const { return m_val[i]; }

		private:
			IC u32 hash(u32 k) const
			{
				// Fibonacci mix. Multiplicative hashing needs the HIGH bits: a product's low bits depend
				// only on the key's low bits, so keys sharing (column mod 2^11, row) would all land on
				// one home slot. The top bits of the 32-bit product depend on every bit of the key.
				return (k * 2654435761u) >> m_shift;
			}

			void grow()
			{
				xr_vector<u32> ok;
				xr_vector<float> ov;
				ok.swap(m_key);
				ov.swap(m_val);
				const u32 n = u32(ok.size()) << 1;
				m_key.assign(n, EMPTY);
				m_val.assign(n, 0.f);
				m_mask = n - 1;
				m_used = 0;
				--m_shift;
				for (u32 i = 0; i < ok.size(); ++i)
					if (EMPTY != ok[i])
						add(ok[i], ov[i]);
			}

			xr_vector<u32> m_key;
			xr_vector<float> m_val;
			u32 m_mask;
			u32 m_used;
			u32 m_shift; // 32 - log2(slots)
		};

		struct cell_entry
		{
			u32 key;
			float ext;
		};

		static bool cell_less(const cell_entry& a, const cell_entry& b) { return a.key < b.key; }
	}

	canopy_oracle& canopy_oracle::instance()
	{
		static canopy_oracle s_oracle;
		return s_oracle;
	}

	void canopy_oracle::on_level_unload()
	{
		xr_vector<u32>().swap(m_col);
		xr_vector<run>().swap(m_runs);
		m_min_sx = m_min_sz = 0;
		m_size_x = m_size_z = 0;
		m_rows = 0;
		m_base_y = 0.f;
		m_g = 0.f;
		m_ready = false;
		m_complete = false;
		m_overcounted = false;
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
			Msg("~ [perception] canopy: no field - trees will not occlude");
			return;
		}

		m_ready = true;

		PDBG_LOG("~ [perception] canopy: %ux%u columns offs (%d,%d) rows %u base %.1f  G %.3f  %.2f MB  bake %.1f ms  %s",
		         m_size_x, m_size_z, m_min_sx, m_min_sz, m_rows, m_base_y, m_g,
		         float(m_col.size() * sizeof(u32) + m_runs.size() * sizeof(run)) / (1024.f * 1024.f),
		         t.GetElapsed_sec() * 1000.f, m_complete ? "complete" : "INCOMPLETE");
	}

	bool canopy_oracle::build()
	{
		static_assert(MAX_ROWS - 1 <= COL_COUNT_MAX, "a column cannot hold more cells than the directory can count");

		canopy_data cd;
		if (!cd.load())
			return false;

		const xr_vector<canopy_model>& models = cd.models();
		const xr_vector<canopy_instance>& insts = cd.instances();

		if (cd.non_tree_leaf())
			Msg("! [perception] canopy: %u leaf-shader visual(s) are not a tree type and were skipped - "
			    "canopy coverage on this level is incomplete", cd.non_tree_leaf());
		if (cd.unresolved_swi())
			Msg("! [perception] canopy: %u progressive-mesh crown(s) with no sliding window - baked at "
				"full index range, leaf area over-counted", cd.unresolved_swi());
		if (cd.no_texture())
			Msg("! [perception] canopy: %u model(s) with no texture - contribute nothing",
				cd.no_texture());

		// unresolved_swi is deliberately not part of this: such a crown is baked at the full index
		// range, which over-counts its leaf area rather than omitting it. Over-counting is the safe
		// direction and never a false-clear, whereas clearing m_complete would disable proxy deferral
		// for every bush on the level and trade a harmless over-count for real double-counting.
		m_complete = cd.non_tree_leaf() == 0 && cd.rejected() == 0 && cd.no_texture() == 0
			&& cd.skipped() == 0;
		m_overcounted = cd.unresolved_swi() != 0;

		if (cd.skipped())
			Msg("! [perception] canopy: %u skip event(s) left leaf geometry unusable - the field does not account for them",
				cd.skipped());
		if (cd.rejected())
			Msg("! [perception] canopy: %u texture(s) unreadable - those crowns contribute nothing",
			    cd.rejected());

		// Per-model object-space box once, then one transformed box per instance. The result sizes the
		// column directory, so it is wanted before any triangle is binned.
		xr_vector<Fbox> local(models.size());
		for (u32 m = 0; m < models.size(); ++m)
		{
			local[m].invalidate();
			for (u32 v = 0; v < models[m].verts.size(); ++v)
				local[m].modify(models[m].verts[v]);
		}

		Fbox world;
		world.invalidate();
		bool any = false;
		for (u32 i = 0; i < insts.size(); ++i)
		{
			const canopy_model& m = models[insts[i].model];
			if (m.verts.empty() || m.ext <= 0.f)
				continue;
			world.modify(local[insts[i].model], insts[i].xform);
			any = true;
		}
		if (!any || !_valid(world.min) || !_valid(world.max))
			return false;

		// Judged in float, before any conversion: one instance matrix with a huge finite translation
		// would make iFloor return INT_MIN, and the signed subtraction below could then wrap to a small
		// count that passes the layout check.
		if (_abs(world.min.x) > MAX_WORLD_COORD || _abs(world.max.x) > MAX_WORLD_COORD
			|| _abs(world.min.y) > MAX_WORLD_COORD || _abs(world.max.y) > MAX_WORLD_COORD
			|| _abs(world.min.z) > MAX_WORLD_COORD || _abs(world.max.z) > MAX_WORLD_COORD)
		{
			Msg("! [perception] canopy: extent %.0f x %.0f x %.0f m exceeds the field layout - no field",
				world.max.x - world.min.x, world.max.z - world.min.z, world.max.y - world.min.y);
			return false;
		}

		m_min_sx = iFloor(world.min.x / SLOT_SIZE);
		m_min_sz = iFloor(world.min.z / SLOT_SIZE);
		const int max_sx = iFloor(world.max.x / SLOT_SIZE);
		const int max_sz = iFloor(world.max.z / SLOT_SIZE);
		m_base_y = float(iFloor(world.min.y / SLOT_SIZE)) * SLOT_SIZE;

		const u32 nx = u32(max_sx - m_min_sx) + 1;
		const u32 nz = u32(max_sz - m_min_sz) + 1;
		const u32 rows = u32(iFloor((world.max.y - m_base_y) / SLOT_SIZE)) + 1;
		if (nx > MAX_COLUMNS_PER_AXIS || nz > MAX_COLUMNS_PER_AXIS || rows > MAX_ROWS - 1)
		{
			Msg("! [perception] canopy: extent %ux%ux%u cells exceeds the field layout - no field",
			    nx, nz, rows);
			return false;
		}
		m_size_x = nx;
		m_size_z = nz;
		m_rows = rows;

		// Bin every card by its centroid. 2 m is the leaf-card scale: binning by centroid is faithful
		// only while the cell is at least as large as the card, which is what fixes the cell size
		// rather than taste (CANOPY-DESIGN 5 and 11).
		bake_map acc;
		acc.init(1u << 19);

		double g_num = 0.0, g_den = 0.0;
		double area_sum = 0.0;
		u32 placed = 0;

		for (u32 i = 0; i < insts.size(); ++i)
		{
			const canopy_instance& inst = insts[i];
			const canopy_model& m = models[inst.model];
			if (m.verts.empty() || m.ext <= 0.f)
				continue;
			++placed;

			for (u32 v = 0; v + 2 < m.verts.size(); v += 3)
			{
				Fvector a, b, c;
				inst.xform.transform_tiny(a, m.verts[v + 0]);
				inst.xform.transform_tiny(b, m.verts[v + 1]);
				inst.xform.transform_tiny(c, m.verts[v + 2]);

				// All three vertices are transformed rather than scaling a precomputed object-space
				// area: the instance transforms carry an anisotropic scale (measured up to 0.84), so
				// no single factor converts object area to world area.
				Fvector e1, e2, n;
				e1.sub(b, a);
				e2.sub(c, a);
				n.crossproduct(e1, e2);
				const float n2 = n.magnitude();
				if (n2 < EPS_S)
					continue;

				const float area = 0.5f * n2;
				const float w = area * m.ext;
				area_sum += area;

				n.div(n2);
				g_num += double(sqrtf(n.x * n.x + n.z * n.z) * w);
				g_den += double(w);

				Fvector cen;
				cen.add(a, b).add(c).div(3.f);
				if (!_valid(cen))
					continue;

				const int cx = iFloor(cen.x / SLOT_SIZE) - m_min_sx;
				const int cz = iFloor(cen.z / SLOT_SIZE) - m_min_sz;
				const int cy = iFloor((cen.y - m_base_y) / SLOT_SIZE);
				if (cx < 0 || cz < 0 || cy < 0 || u32(cx) >= m_size_x || u32(cz) >= m_size_z
					|| u32(cy) >= rows)
					continue;

				const u32 col = u32(cz) * m_size_x + u32(cx);
				acc.add(col * MAX_ROWS + u32(cy), w);
			}
		}

		if (!acc.used() || g_den <= 0.0)
			return false;

		m_g = float(g_num / g_den) * (2.f / PI);

		xr_vector<cell_entry> cells;
		cells.reserve(acc.used());
		for (u32 i = 0; i < acc.slots(); ++i)
		{
			if (bake_map::EMPTY == acc.key_at(i))
				continue;
			if (acc.val_at(i) < MIN_CELL_EXT)
				continue;
			cell_entry e;
			e.key = acc.key_at(i);
			e.ext = acc.val_at(i);
			cells.push_back(e);
		}
		if (cells.empty())
			return false;

		// Sorting by key groups the cells by column and orders them by row inside it, which is exactly
		// the layout the directory addresses.
		std::sort(cells.begin(), cells.end(), cell_less);

		if (cells.size() > COL_OFFSET_MAX)
		{
			Msg("! [perception] canopy: %u cells exceed the %u the directory can address - no field",
			    u32(cells.size()), COL_OFFSET_MAX);
			return false;
		}

		m_col.assign(u32(m_size_x) * m_size_z, 0);
		m_runs.reserve(cells.size());

		u32 dropped = 0;
		u32 i = 0;
		while (i < cells.size())
		{
			const u32 col = cells[i].key / MAX_ROWS;
			u32 j = i;
			while (j < cells.size() && cells[j].key / MAX_ROWS == col)
				++j;

			u32 count = j - i;
			if (count > COL_COUNT_MAX)
			{
				dropped += count - COL_COUNT_MAX;
				count = COL_COUNT_MAX;
			}

			m_col[col] = (u32(m_runs.size()) << COL_COUNT_BITS) | count;
			for (u32 k = 0; k < count; ++k)
			{
				run r;
				r.ext = cells[i + k].ext;
				r.y = u8(cells[i + k].key % MAX_ROWS);
				m_runs.push_back(r);
			}
			i = j;
		}

		// Unreachable while a column holds at most one cell per row and rows are capped below
		// COL_COUNT_MAX (the static_assert in build()); kept for the day MAX_ROWS is raised. Dropped
		// cells are vegetation the field no longer accounts for.
		if (dropped)
			m_complete = false;

		if (dropped)
			Msg("! [perception] canopy: %u cell(s) past the %u-per-column limit were dropped", dropped,
			    COL_COUNT_MAX);

#ifdef PERCEPTION_DEBUG
		m_stats.cells = u32(m_runs.size());
		m_stats.instances = placed;
		m_stats.models = u32(models.size());
		m_stats.leaf_area = float(area_sum);
		PDBG_LOG("~ [perception] canopy: %u model triangle(s), %.0f m2 instanced card area, %u cell(s) from %u slot(s)",
		         cd.triangles(), float(area_sum), u32(m_runs.size()), acc.slots());
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
		y_lo = m_base_y + float(m_runs[first].y) * SLOT_SIZE;
		y_hi = m_base_y + float(m_runs[first + count - 1].y) * SLOT_SIZE + SLOT_SIZE;
		return true;
	}

	void canopy_oracle::march(const Fvector& from, const Fvector& to, march_result& out,
	                          xr_vector<veg_chord>* chords) const
	{
		out.tau = 0.f;
		out.cells = 0;
		if (!m_ready)
			return;

		PDBG_ONLY(++m_stats.marches);

		// Whole-field reject, a conservative box test with no clipping. Without it a ray that never
		// touches the field still pays the full slot walk (up to 4096 steps) to fail every bounds
		// check, uncached, on every trace.
		const float field_top = m_base_y + float(m_rows) * SLOT_SIZE;
		if (_max(from.y, to.y) < m_base_y || _min(from.y, to.y) > field_top)
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

		const bool climbs = _abs(dir.y) > EPS_S;
		const float inv_y = climbs ? (1.f / dir.y) : 0.f;

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

			const float t_enter = w.t_enter();
			const float t_exit = w.t_exit();

			// The ray's height WHERE IT CROSSES THIS COLUMN. Over a whole segment a long shot spans
			// tens of metres vertically and overlaps nearly every column's canopy band, so a global
			// test rejects almost nothing.
			const float ya = from.y + dir.y * t_enter;
			const float yb = from.y + dir.y * t_exit;
			const float y_min = _min(ya, yb);
			const float y_max = _max(ya, yb);

			const u32 first = c >> COL_COUNT_BITS;
			if (y_max < m_base_y + float(m_runs[first].y) * SLOT_SIZE
				|| y_min > m_base_y + float(m_runs[first + count - 1].y) * SLOT_SIZE + SLOT_SIZE)
			{
				PDBG_ONLY(++m_stats.columns_skipped);
				continue;
			}

			for (u32 k = 0; k < count; ++k)
			{
				const run& r = m_runs[first + k];
				const float lo = m_base_y + float(r.y) * SLOT_SIZE;
				const float hi = lo + SLOT_SIZE;
				if (y_max < lo || y_min > hi)
					continue;

				float t0 = t_enter, t1 = t_exit;
				if (climbs)
				{
					float ta = (lo - from.y) * inv_y;
					float tb = (hi - from.y) * inv_y;
					if (ta > tb)
						std::swap(ta, tb);
					t0 = _max(t0, ta);
					t1 = _min(t1, tb);
				}

				const float path = t1 - t0;
				if (path <= 0.f)
					continue;

				// Leaf-area-density extinction, with the density measured off real triangles: the
				// chance of meeting a card per metre is G * (area / volume), and the card's own
				// -ln(1-alpha) is already folded into ext at bake time.
				const float tau = m_g * (r.ext / CELL_VOLUME) * path;
				out.tau += tau;
				++out.cells;
				PDBG_ONLY(++m_stats.cells_entered);

				if (chords)
				{
					veg_chord ch;
					ch.enter.mad(from, dir, t0);
					ch.exit.mad(from, dir, t1);
					ch.blocked = 1.f - expf(-tau);
					chords->push_back(ch);
				}
			}
		}
	}
}
