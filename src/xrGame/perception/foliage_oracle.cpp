#include "stdafx.h"
#include <algorithm>
#include "foliage_oracle.h"
#include "detail_data.h"
#include "foliage_profile.h"
#include "slot_walk.h"
#include "perception_debug.h"

namespace perception
{
	static const u32 MAX_FIELD_SLOTS = 8192 * 8192;

	// Mean fraction of an upright face's area a horizontal ray sees, averaged over the instance's
	// random yaw: <|cos|> over a full turn = 2/pi.
	static const float FOLIAGE_G = 0.6366f;
	static const u32 TUFT_STEPS_MIN = 2;
	static const u32 TUFT_STEPS_MAX = 16;

	foliage_oracle& foliage_oracle::instance()
	{
		static foliage_oracle s_oracle;
		return s_oracle;
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
		m_ready = false;
	}

	void foliage_oracle::on_level_load()
	{
		PDBG_SCOPE("foliage: total bake");
		on_level_unload();

		detail_data dd;
		{
			PDBG_SCOPE("foliage: parse level.details");
			if (!dd.load())
				return;
		}

		build_foliage_profiles(dd, m_alpha_ref, m_profiles);

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

			// The grid bounds the height band grass could occupy, so a ray that
			// passes well above or below a slot is rejected before anything is resolved. The band is
			// the slot's terrain relief plus the highest top any of its models reaches at the largest
			// scale placement draws, because grass grows on the surface and the surface itself spans
			// `relief` inside one 2 m slot. The detail height slider is applied at query time.
			detail_slot_desc s_desc;
			for (u32 i = 0; i < n; ++i)
			{
				dd.slot(i, s_desc);

				float tallest = 0.f;
				for (u32 m = 0; m < 4; ++m)
				{
					if (DETAIL_ID_EMPTY == s_desc.id[m] || s_desc.id[m] >= m_profiles.size())
						continue;
					const foliage_model_profile& p = m_profiles[s_desc.id[m]];
					const float top = p.density.valid() ? p.density.y_max : p.height;
					tallest = _max(tallest, top * p.max_placement_scale);
				}

				cell& c = m_field[i];
				c.tallest = u8(_min(iCeil(tallest * 10.f), 255));
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
		m_detail_height = ::Render ? ::Render->detail_height() : 1.f;

		m_ready = true;

		PDBG_LOG("~ [perception] foliage: v%d grid %dx%d offs (%d,%d) models %d  bounds %.2f MB  renderer places: %s",
		         dd.version(), m_size_x, m_size_z, m_offs_x, m_offs_z, u32(dd.models().size()),
		         float(m_field.size() * sizeof(cell)) / (1024.f * 1024.f),
		         m_renderer_places ? "yes" : "no");
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

		y_lo = float(c.slab_base) * 0.2f - 200.f;
		y_hi = y_lo + band_height(c);
		return true;
	}

	float foliage_oracle::band_height(const cell& c) const
	{
		float relief = float(c.slab_h) * 0.1f;
		if (m_max_slab_h > 0.f)
			relief = _min(relief, m_max_slab_h);
		return relief + float(c.tallest) * 0.1f * m_detail_height;
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
		// would keep serving pre-change instances for the rest of the session. Placement reads the
		// height slider live, so a change to it alone re-scales every instance resolved from then on.
		if (::Render && (::Render->detail_generation() != m_detail_gen || ::Render->detail_height() != m_detail_height))
		{
			const u32 dropped = u32(m_resolved.size());
			m_resolved.clear();
			m_instance_count = 0;
			m_detail_gen = ::Render->detail_generation();
			m_detail_height = ::Render->detail_height();
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

			const float h = band_height(c);
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

		// Projected share of upright and of flat faces along this direction (TUFT-DENSITY-DESIGN 2.2).
		const float k_up = FOLIAGE_G * _sqrt(a);
		const float k_flat = _abs(dir.y);

		for (u32 i = 0; i < r.items.size(); ++i)
		{
			const detail_instance& inst = r.items[i];
			if (inst.model >= m_profiles.size() || inst.scale <= 0.f)
				continue;
			const tuft_density& d = m_profiles[inst.model].density;
			if (!d.valid())
				continue;

			const float s = inst.scale;
			const float rad = d.radius * s;
			const float y_lo = inst.pos.y + d.y_min * s;
			const float y_hi = inst.pos.y + d.y_max * s;

			// Ray versus the model's bounding vertical cylinder around the instance origin.
			const float dx = from.x - inst.pos.x;
			const float dz = from.z - inst.pos.z;
			const float b = 2.f * (dx * dir.x + dz * dir.z);
			const float cc = dx * dx + dz * dz - rad * rad;
			const float disc = b * b - 4.f * a * cc;
			if (disc <= 0.f)
				continue;

			const float sq = _sqrt(disc);
			float t0 = _max((-b - sq) / (2.f * a), 0.f);
			float t1 = _min((-b + sq) / (2.f * a), len);

			if (_abs(dir.y) > EPS_S)
			{
				float ta = (y_lo - from.y) / dir.y;
				float tb = (y_hi - from.y) / dir.y;
				if (ta > tb)
					std::swap(ta, tb);
				t0 = _max(t0, ta);
				t1 = _min(t1, tb);
			}
			else if (from.y < y_lo || from.y > y_hi)
				continue;
			if (t1 <= t0)
				continue;

			// Midpoint rule over the chord, about one step per radial bin.
			const float chord_len = t1 - t0;
			const u32 steps = _min(_max(u32(iCeil(chord_len * float(TUFT_NR) / rad)), TUFT_STEPS_MIN), TUFT_STEPS_MAX);
			const float dt = chord_len / float(steps);
			const float inv_s = 1.f / s;

			float tau = 0.f;
			for (u32 k = 0; k < steps; ++k)
			{
				Fvector q;
				q.mad(from, dir, t0 + (float(k) + .5f) * dt);
				q.sub(inst.pos);
				q.mul(inv_s);
				tau += d.extinction(q, k_up, k_flat);
			}
			// World density is the model-space density over the scale: area goes as s^2, volume as s^3.
			tau *= dt * inv_s;
			if (tau <= 0.f)
				continue;

			out.tau += tau;
			++out.crossings;

			if (chords)
			{
				chord ch;
				ch.enter.mad(from, dir, t0);
				ch.exit.mad(from, dir, t1);
				ch.blocked = 1.f - expf(-tau);
				ch.origin = inst.pos;
				ch.radius = rad;
				ch.y_lo = y_lo;
				ch.y_hi = y_hi;
				ch.core_radius = d.r90 * s;
				ch.core_top = inst.pos.y + d.top95 * s;
				ch.scale = s;
				ch.tau = tau;
				ch.model = inst.model;
				chords->push_back(ch);
			}
		}
	}
}
