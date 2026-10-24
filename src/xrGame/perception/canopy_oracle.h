#pragma once

#include "../../build_config_defines.h"
#include "occlusion_types.h"

namespace perception
{
	// Tree canopy as an AI-visible occluder.
	//
	// Leaf cards are absent from level.cform entirely - the compiler shader `flora\leaf` is
	// flCollision = 0 (shaders_xrlc.xr) - so no ray query can ever hit them. This field is the only
	// thing that makes a crown block a sight line.
	//
	// Baked once at level load from the real leaf triangles and their real atlas alpha, as
	// extinction-weighted leaf area per 2 m cell. There is nothing to resolve on demand and nothing to
	// cache: unlike grass the whole field is resident, so the march is a plain grid walk with no
	// per-frame budget and no unresolved state. See CANOPY-DESIGN.
	class canopy_oracle
	{
	public:
		static canopy_oracle& instance();

		void on_level_load();
		void on_level_unload();
		bool ready() const { return m_ready; }

		// The field accounts for every piece of leaf geometry found; when false, some vegetation is
		// missing from it and the collision proxies must keep contributing.
		bool complete() const { return m_ready && m_complete; }

		// Some progressive-mesh crowns had no sliding window and were baked with every LOD summed.
		bool overcounted() const { return m_ready && m_overcounted; }

		struct march_result
		{
			float tau;  // optical depth of canopy on the path
			u16 cells;  // occupied cells actually crossed
		};

		void march(const Fvector& from, const Fvector& to, march_result& out,
		           xr_vector<veg_chord>* chords = 0) const;

		// Vertical extent canopy could occupy at a world XZ. false = this column holds none.
		bool band_at(float x, float z, float& y_lo, float& y_hi) const;

#ifdef PERCEPTION_DEBUG
		struct stats
		{
			u32 cells;      // occupied cells in the field
			u32 instances;  // tree crowns baked
			u32 models;     // distinct leaf geometries
			float bake_ms;
			float leaf_area; // summed card area, m2

			u32 marches;
			u32 columns_walked; // columns crossed that hold canopy
			u32 columns_skipped; // ...of those, rejected because the ray missed the band
			u32 cells_entered;
		};

		// Only the per-march counters; everything else in stats is bake-time and survives.
		void reset_stats() const
		{
			m_stats.marches = 0;
			m_stats.columns_walked = 0;
			m_stats.columns_skipped = 0;
			m_stats.cells_entered = 0;
		}

		const stats& get_stats() const { return m_stats; }
#endif

	private:
		// One occupied cell. 8 bytes; 223 k of them on Escape, 180 k on Red Forest.
		struct run
		{
			float ext; // extinction-weighted leaf area in this cell, m2
			u8 y;      // cell row; world Y = m_base_y + y * SLOT_SIZE
		};

		bool build();

		// Column directory, one entry per 2 m XZ column: offset into m_runs in the high 24 bits,
		// run count in the low 8.
		static const u32 COL_COUNT_BITS = 8;
		static const u32 COL_COUNT_MAX = (1u << COL_COUNT_BITS) - 1;
		static const u32 COL_OFFSET_MAX = (1u << (32 - COL_COUNT_BITS)) - 1;

		u32 column(int gx, int gz) const { return m_col[u32(gz) * m_size_x + u32(gx)]; }

		xr_vector<u32> m_col;
		xr_vector<run> m_runs;

		int m_min_sx = 0; // world slot index of column (0,0)
		int m_min_sz = 0;
		u32 m_size_x = 0;
		u32 m_size_z = 0;
		u32 m_rows = 0;      // cell rows above m_base_y; the row index itself is u8 in `run`
		float m_base_y = 0.f;

		// Mean |n . d| over the level's leaf cards for d uniform over horizontal azimuth, weighted by
		// area * extinction (the same weight the cell sums carry). For a card of unit normal n that
		// average is (2/pi) * sqrt(n.x^2 + n.z^2), the same 2/pi foliage_oracle's FOLIAGE_G derives
		// from. Measured at bake time rather than assumed: 0.469 on Escape, 0.472 on Red Forest.
		float m_g = 0.f;

		bool m_ready = false;
		bool m_complete = false;
		bool m_overcounted = false;

#ifdef PERCEPTION_DEBUG
		mutable stats m_stats;
#endif
	};
}
