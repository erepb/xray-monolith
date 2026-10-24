#include "stdafx.h"
#include <algorithm>
#include "foliage_oracle.h"
#include "detail_data.h"
#include "foliage_profile.h"
#include "slot_walk.h"
#include "perception_debug.h"

namespace perception
{
	static const u32 LEVEL_CHUNK_SHADERS = 2; // fsL_SHADERS, xrEngine/xrLevel.h
	static const u32 MAX_FIELD_SLOTS = 8192 * 8192;

	// Mean fraction of a vertical quad's area a horizontal ray sees, averaged over the instance's
	// random yaw: <|cos|> over a full turn = 2/pi. Crossed pairs give the same figure, the two
	// perpendicular halves averaging to it as well.
	static const float FOLIAGE_G = 0.6366f;

	foliage_oracle& foliage_oracle::instance()
	{
		static foliage_oracle s_oracle;
		return s_oracle;
	}

	float foliage_oracle::leafiness(veg_class c) const
	{
		return (c < veg_class_count) ? m_leafiness[c] : 1.f;
	}

	float foliage_oracle::gap_fraction(veg_class c) const
	{
		return (c < veg_class_count) ? m_gap[c] : 0.f;
	}

	void foliage_oracle::on_level_unload()
	{
		xr_vector<cell>().swap(m_field);
		xr_vector<foliage_model_profile>().swap(m_profiles);
		m_resolved.clear();
		m_instance_count = 0;
		m_budget_left = 0;
		m_renderer_places = false;
		PDBG_ONLY(ZeroMemory(&m_stats, sizeof(m_stats)));
		m_size_x = m_size_z = 0;
		m_offs_x = m_offs_z = 0;
		for (u32 i = 0; i < veg_class_count; ++i)
		{
			m_leafiness[i] = 1.f;
			m_gap[i] = 0.f;
		}
		m_max_slab_h = 0.f;
		m_resolve_budget = 0;
		m_cache_max = 0;
		m_ready = false;
	}

	void foliage_oracle::on_level_load()
	{
		PDBG_SCOPE("foliage: total bake");
		on_level_unload();

		scan_leafiness();

		detail_data dd;
		{
			PDBG_SCOPE("foliage: parse level.details");
			if (!dd.load())
				return;
		}

		build_foliage_profiles(dd, m_profiles);

		m_size_x = dd.size_x();
		m_size_z = dd.size_z();
		m_offs_x = dd.offs_x();
		m_offs_z = dd.offs_z();

		{
			PDBG_SCOPE("foliage: bake slot bounds");

			const u64 n64 = u64(dd.size_x()) * dd.size_z();
			if (!n64 || n64 > u64(MAX_FIELD_SLOTS))
			{
				Msg("! [perception] foliage: field size implausible: %ux%u", dd.size_x(), dd.size_z());
				return;   // m_ready stays false
			}
			const u32 n = u32(n64);
			m_field.resize(n);

			// The grid keeps one job now: bound the height band grass could occupy, so a ray that
			// passes well above or below a slot is rejected before anything is resolved. The band is
			// the slot's terrain relief plus the tallest model the slot references, because grass
			// grows on the surface and the surface itself spans `relief` inside one 2 m slot.
			detail_slot_desc s_desc;
			for (u32 i = 0; i < n; ++i)
			{
				dd.slot(i, s_desc);

				float tallest = 0.f;
				for (u32 m = 0; m < 4; ++m)
				{
					if (DETAIL_ID_EMPTY == s_desc.id[m] || s_desc.id[m] >= m_profiles.size())
						continue;
					tallest = _max(tallest, m_profiles[s_desc.id[m]].height);
				}

				cell& c = m_field[i];
				c.tallest = u8(_min(iFloor(tallest * 10.f + 0.5f), 255));
				c.slab_h = s_desc.y_height;
				c.slab_base = s_desc.y_base;
				c.pad = 0;
			}
		}

		// Null means R1/R2/R3 or details disabled: no grass is drawn, so there is nothing to hide
		// behind and the foliage channel stays out of the way entirely. Probed once here rather than
		// per trace so the answer cannot change mid-frame.
		{
			xr_vector<detail_instance> probe;
			m_renderer_places = ::Render && ::Render->resolve_detail_slot(-m_offs_x, -m_offs_z, probe);
		}

		// Baseline for on_frame()'s change check - a fresh load already reflects current placement.
		m_detail_gen = ::Render ? ::Render->detail_generation() : 0;

		m_ready = true;

		PDBG_LOG("~ [perception] foliage: v%d grid %dx%d offs (%d,%d) models %d  bounds %.2f MB  renderer places: %s",
		         dd.version(), m_size_x, m_size_z, m_offs_x, m_offs_z, u32(dd.models().size()),
		         float(m_field.size() * sizeof(cell)) / (1024.f * 1024.f),
		         m_renderer_places ? "yes" : "no");
	}

	void foliage_oracle::scan_leafiness()
	{
		PDBG_SCOPE("foliage: leafiness scan");

		float sum[veg_class_count] = {0.f, 0.f};
		float gap_sum[veg_class_count] = {0.f, 0.f};
		u32 count[veg_class_count] = {0, 0};
		u32 rejected[veg_class_count] = {0, 0};
		xr_vector<shared_str> seen;

		if (!FS.exist("$level$", "level"))
		{
			PDBG_LOG("~ [perception] foliage: level file absent - skipping leafiness scan");
			return;
		}

		string_path fn;
		FS.update_path(fn, "$level$", "level");
		IReader* fs = FS.r_open(fn);
		if (!fs)
		{
			Msg("! [perception] foliage: failed to open level file - skipping leafiness scan");
			return;
		}

		IReader* chunk = open_chunk_checked(fs, LEVEL_CHUNK_SHADERS);
		if (chunk)
		{
			if (chunk->elapsed() < 4)
			{
				Msg("! [perception] foliage: level shaders chunk too short - skipping leafiness scan");
				chunk->close();
				FS.r_close(fs);
				return;
			}
			const u32 n = chunk->r_u32();
			for (u32 i = 0; i < n && chunk->elapsed() > 0; ++i)
			{
				string512 entry;
				if (!read_stringz(chunk, entry, sizeof(entry)))
					break;
				if (!entry[0])
					continue;

				// "<shader>/<texture>[,<lmap>...]"
				LPSTR slash = strchr(entry, '/');
				if (!slash)
					continue;
				*slash = 0;
				LPSTR tex = slash + 1;
				LPSTR comma = strchr(tex, ',');
				if (comma)
					*comma = 0;

				if (0 != strncmp(entry, "flora", 5))
					continue;

				// flora\trunk_wave is bark: opaque, and untouched by the winter/bare-branch
				// retextures this metric exists to detect
				if (strstr(entry, "trunk"))
					continue;

				// a texture referenced by more than one flora shader (e.g. leaf/leaf_wave) must only
				// contribute once, or the class mean gets weighted by shader multiplicity
				const shared_str tex_name = tex;
				bool dup = false;
				for (u32 s = 0; s < seen.size(); ++s)
					if (seen[s] == tex_name)
					{
						dup = true;
						break;
					}
				if (dup)
					continue;

				const veg_class c = strstr(tex, "bush") ? veg_bush : veg_tree;

				dds_info info;
				uv_rect whole;
				whole.invalidate();
				float a = 0.f, g = 0.f;
				if (!texture_mean_alpha(tex, whole, a, &info, &g))
				{
					++rejected[c];
					continue;
				}

				// vanilla still references deprecated *_frond* entries whose textures are 2x2 stubs;
				// an image this small carries no leaf-density information, so it must not vote
				if (info.width < 8 || info.height < 8)
				{
					++rejected[c];
					continue;
				}

				seen.push_back(tex_name);

				sum[c] += a;
				gap_sum[c] += g;
				count[c] += 1;

				PDBG_LOG("~ [perception] foliage:   flora tex '%s' shader '%s' class %s alpha=%.3f gap=%.3f",
				         tex, entry, (veg_bush == c) ? "bush" : "tree", a, g);
			}
			chunk->close();
		}
		else
		{
			Msg("! [perception] foliage: level shaders chunk missing or invalid - skipping leafiness scan");
		}
		FS.r_close(fs);

		for (u32 i = 0; i < veg_class_count; ++i)
		{
			if (count[i])
			{
				m_leafiness[i] = sum[i] / float(count[i]);
				m_gap[i] = gap_sum[i] / float(count[i]);
			}
			else if (rejected[i])
			{
				// every flora texture for this class was unusable (degenerate stubs or unreadable):
				// the vegetation renders as nothing, so it must not inherit the full-concealment
				// default that "no vegetation on this level" relies on
				m_leafiness[i] = 0.f;
				// bare, not merely sparse: nothing is left to be unevenly distributed
				m_gap[i] = 1.f;
				Msg("! [perception] foliage: all %d %s textures unusable - treating class as bare",
				    rejected[i], (veg_bush == veg_class(i)) ? "bush" : "tree");
			}
		}

		PDBG_LOG("~ [perception] foliage: leafiness bush=%.3f gap=%.3f (%d tex, %d rejected) tree=%.3f gap=%.3f (%d tex, %d rejected)",
		         m_leafiness[veg_bush], m_gap[veg_bush], count[veg_bush], rejected[veg_bush],
		         m_leafiness[veg_tree], m_gap[veg_tree], count[veg_tree], rejected[veg_tree]);
	}

	bool foliage_oracle::band_at(float x, float z, float& y_lo, float& y_hi) const
	{
		if (!m_ready)
			return false;

		// Slot sx spans world X in [sx*SLOT_SIZE, sx*SLOT_SIZE+SLOT_SIZE) per DetailManager_CACHE.cpp
		// and DetailManager_Decompress.cpp - no half-slot bias.
		const int gx = iFloor(x / SLOT_SIZE) + m_offs_x;
		const int gz = iFloor(z / SLOT_SIZE) + m_offs_z;
		if (gx < 0 || gz < 0 || u32(gx) >= m_size_x || u32(gz) >= m_size_z)
			return false;

		const cell& c = m_field[u32(gz) * m_size_x + u32(gx)];
		if (!c.tallest)
			return false;

		float h = float(c.slab_h) * 0.1f + float(c.tallest) * 0.1f;
		if (m_max_slab_h > 0.f)
			h = _min(h, m_max_slab_h);

		y_lo = float(c.slab_base) * 0.2f - 200.f;
		y_hi = y_lo + h;
		return true;
	}

	void foliage_oracle::on_frame()
	{
		m_budget_left = m_resolve_budget;

		// CLevel::OnFrame runs while paused (perception_debug.cpp's on_frame guards on Device.Paused()
		// for the same reason) - freeze the eviction clock too, or every last_used looks stale on resume.
		if (!Device.Paused())
			++m_frame;

		// The renderer rebuilds Details (and re-places every instance) when detail density/height or
		// radius changes on vid_restart. Nothing else tells the oracle that happened, so a stale cache
		// would keep serving pre-change instances for the rest of the session.
		if (::Render && ::Render->detail_generation() != m_detail_gen)
		{
			const u32 dropped = u32(m_resolved.size());
			m_resolved.clear();
			m_instance_count = 0;
			m_detail_gen = ::Render->detail_generation();
			Msg("~ [perception] foliage: detail placement changed - dropped %u resolved slot(s)", dropped);
		}

		evict_if_needed();
	}

	void foliage_oracle::evict_if_needed() const
	{
		if (m_resolved.size() <= m_cache_max)
			return;

		// Trims below the cap, so the O(N) pass below runs once per many frames rather than every frame.
		const u32 trim = m_cache_max - m_cache_max / 8;

		// Coarse LRU. Placement never changes for the level's lifetime, so eviction only ever costs a
		// re-resolve - it can never cost correctness.
		const u32 cutoff = (m_frame > 60) ? (m_frame - 60) : 0;
		for (resolved_map::iterator it = m_resolved.begin(); it != m_resolved.end();)
		{
			if (it->second.last_used < cutoff)
			{
				m_instance_count -= u32(it->second.items.size());
				m_resolved.erase(it++);
			}
			else
				++it;
		}

		// Age alone does not guarantee landing back under the cap: every cache hit refreshes
		// last_used, so a working set held entirely within the last 60 frames survives the pass above
		// intact. Finish the job oldest-first. The oldest are found by a single selection pass rather
		// than by rescanning the map per eviction.
		if (m_resolved.size() > m_cache_max)
		{
			const u32 excess = u32(m_resolved.size() - trim);

			xr_vector<u32> stamps;
			stamps.reserve(m_resolved.size());
			for (resolved_map::iterator it = m_resolved.begin(); it != m_resolved.end(); ++it)
				stamps.push_back(it->second.last_used);

			std::nth_element(stamps.begin(), stamps.begin() + excess, stamps.end());
			const u32 threshold = stamps[excess];

			// Ties at the threshold would otherwise over-evict, since more than `excess` entries can
			// share the same last_used stamp.
			for (resolved_map::iterator it = m_resolved.begin();
			     it != m_resolved.end() && m_resolved.size() > trim;)
			{
				if (it->second.last_used <= threshold)
				{
					m_instance_count -= u32(it->second.items.size());
					m_resolved.erase(it++);
				}
				else
					++it;
			}
		}
	}

	const foliage_oracle::resolved* foliage_oracle::slot_instances(int gx, int gz) const
	{
		const u32 key = u32(gz) * m_size_x + u32(gx);

		resolved_map::iterator it = m_resolved.find(key);
		if (it != m_resolved.end())
		{
			it->second.last_used = m_frame;
			return &it->second;
		}

		if (!m_budget_left)
			return 0; // caller treats this as occluding

		--m_budget_left;

		resolved& r = m_resolved[key];
		r.last_used = m_frame;

#ifdef PERCEPTION_DEBUG
		CTimer t;
		t.Start();
#endif
		r.items.reserve(32); // ~24 instances/slot measured on Escape at default density
		::Render->resolve_detail_slot(gx - m_offs_x, gz - m_offs_z, r.items);
#ifdef PERCEPTION_DEBUG
		m_stats.resolve_us += t.GetElapsed_sec() * 1000000.f;
		++m_stats.resolve_requests;
#endif

		m_instance_count += u32(r.items.size());

		return &r;
	}

	void foliage_oracle::march(const Fvector& from, const Fvector& to, march_result& out,
	                           xr_vector<chord>* chords) const
	{
		out.tau = 0.f;
		out.crossings = 0;
		out.unresolved = 0;
		if (!m_ready || !m_renderer_places)
			return;

		PDBG_ONLY(++m_stats.marches);

		// Slot boundaries are the natural step: there is nothing to sample between them, only
		// instances to intersect.
		slot_walk w(from, to);
		if (!w.valid())
			return;
		const Fvector& dir = w.dir();
		const float len = w.len();
		while (w.next())
		{
			const int gx = w.sx() + m_offs_x;
			const int gz = w.sz() + m_offs_z;
			if (gx < 0 || gz < 0 || u32(gx) >= m_size_x || u32(gz) >= m_size_z)
				continue;

			const cell& c = m_field[u32(gz) * m_size_x + u32(gx)];
			if (!c.tallest)
				continue;

			PDBG_ONLY(++m_stats.slots_walked);

			float h = float(c.slab_h) * 0.1f + float(c.tallest) * 0.1f;
			if (m_max_slab_h > 0.f)
				h = _min(h, m_max_slab_h);
			const float y_lo = float(c.slab_base) * 0.2f - 200.f;

			// The ray's height WHERE IT CROSSES THIS SLOT, not over its whole length. The difference
			// decides everything on long shots: a 200 m ray that drops 30 m has a 30 m tall global Y
			// span, which overlaps nearly every slot's grass band, so a global test rejects almost
			// nothing and every slot gets resolved. Per-slot, the span is centimetres and a ray flying
			// above the canopy rejects the whole level for the cost of the DDA walk.
			const float ya = from.y + dir.y * w.t_enter();
			const float yb = from.y + dir.y * w.t_exit();

			if (_max(ya, yb) < y_lo || _min(ya, yb) > y_lo + h)
			{
				PDBG_ONLY(++m_stats.slots_above);
				continue;
			}

			const resolved* r = slot_instances(gx, gz);
			if (!r)
			{
				// Budget spent. Occluding, not clear: a false sighting latches in visual memory and
				// propagates to the squad, while a missed one costs a frame and self-corrects
				// (FOLIAGE-DISCRETE-DESIGN 5).
				++out.unresolved;
				out.tau = flt_max;
				PDBG_ONLY(++m_stats.budget_denied);
			}
			else
			{
				PDBG_ONLY(++m_stats.slots_entered);
				accumulate_slot(*r, from, dir, len, out, chords);
			}
		}
	}

	void foliage_oracle::accumulate_slot(const resolved& r, const Fvector& from, const Fvector& dir,
	                                     float len, march_result& out, xr_vector<chord>* chords) const
	{
		// A ray with no horizontal component cannot travel through grass, only clip it; the cylinder
		// solve is degenerate there and the case is not worth a separate branch.
		const float a = dir.x * dir.x + dir.z * dir.z;
		if (a < EPS_S)
			return;

		for (u32 i = 0; i < r.items.size(); ++i)
		{
			const detail_instance& inst = r.items[i];
			if (inst.model >= m_profiles.size())
				continue;
			const foliage_model_profile& p = m_profiles[inst.model];
			if (p.radius <= 0.f || p.height <= 0.f || p.opacity <= 0.f)
				continue;

			const float rad = p.radius * inst.scale;
			const float hgt = p.height * inst.scale;

			// Ray versus vertical cylinder. Rotationally symmetric, which is why neither the random
			// yaw nor the terrain-align rotation crosses the render boundary at all.
			const float dx = from.x - inst.pos.x;
			const float dz = from.z - inst.pos.z;
			const float b = 2.f * (dx * dir.x + dz * dir.z);
			const float cc = dx * dx + dz * dz - rad * rad;
			const float disc = b * b - 4.f * a * cc;
			if (disc <= 0.f)
				continue;

			const float sq = _sqrt(disc);
			float t0 = (-b - sq) / (2.f * a);
			float t1 = (-b + sq) / (2.f * a);
			if (t1 <= 0.f || t0 >= len)
				continue;
			t0 = _max(t0, 0.f);
			t1 = _min(t1, len);
			if (t1 <= t0)
				continue;

			// Vertical overlap, against the tuft's own extent rather than the slot band.
			const float y0 = from.y + dir.y * t0;
			const float y1 = from.y + dir.y * t1;
			if (_max(y0, y1) < inst.pos.y || _min(y0, y1) > inst.pos.y + hgt)
				continue;

			// Fraction of this tuft's cross-section that is actually opaque: mean projected quad area
			// over the instance's yaw, spread across the footprint the cylinder presents.
			const float projected = p.area * inst.scale * inst.scale * FOLIAGE_G;
			float blocked = projected / (hgt * 2.f * rad);
			clamp(blocked, 0.f, 1.f);

			float t = 1.f - p.opacity * blocked;
			clamp(t, 1e-4f, 1.f);

			out.tau += -logf(t);
			++out.crossings;

			if (chords)
			{
				chord ch;
				ch.enter.mad(from, dir, t0);
				ch.exit.mad(from, dir, t1);
				ch.blocked = p.opacity * blocked;
				chords->push_back(ch);
			}
		}
	}
}
