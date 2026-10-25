#pragma once

#include "../../build_config_defines.h"
#include "occlusion_types.h"
#include "coverage_lod.h"

namespace perception
{
	struct canopy_model;

	// Tree canopy as an AI-visible occluder.
	//
	// Leaf cards are absent from level.cform entirely - the compiler shader `flora\leaf` is
	// flCollision = 0 (shaders_xrlc.xr) - so no ray query can ever hit them. This oracle is the only
	// thing that makes a crown block a sight line.
	//
	// Built once at level load from the real leaf triangles and their real atlas coverage. A ray sums
	// -ln(1 - a) over the cards it actually pierces, where a is the leaf share of the texture around the
	// hit point over the sight line's width there. Each placed crown is intersected in its own object
	// space through a per-model BVH, found through a 2 m column index. See CANOPY-DISCRETE-DESIGN 12.
	class canopy_oracle
	{
	public:
		static canopy_oracle& instance();

		// Read at the next level load. finest_cell: metres, the narrowest sight line the coverage
		// pyramids resolve. alpha_ref: 0..1, the cutoff for leaf textures whose alpha is not one bit.
		void set_bake_params(float finest_cell, float alpha_ref)
		{
			m_finest_cell = finest_cell;
			m_alpha_ref = alpha_ref;
		}

		void on_level_load();
		void on_level_unload();
		bool ready() const { return m_ready; }

		// Every piece of leaf geometry found is accounted for; when false, some vegetation is missing and
		// the collision proxies must keep contributing.
		bool complete() const { return m_ready && m_complete; }

		// Some progressive-mesh crowns had no sliding window and were built with every LOD summed.
		bool overcounted() const { return m_ready && m_overcounted; }

		struct march_result
		{
			float tau;  // optical depth of canopy on the path
			u16 crowns; // crowns whose cards the path pierced
		};

		// The sight line is `eye` metres wide at `from` and `target` metres wide at `to`; a card nearer
		// to `from` than near_plane is not counted.
		struct sight_cone
		{
			float eye;
			float target;
			float near_plane;
		};

		void march(const Fvector& from, const Fvector& to, const sight_cone& cone, march_result& out,
		           xr_vector<veg_chord>* chords = 0, xr_vector<canopy_hit>* hits = 0) const;

		// Vertical extent canopy could occupy at a world XZ. false = this column holds none.
		bool band_at(float x, float z, float& y_lo, float& y_hi) const;

#ifdef PERCEPTION_DEBUG
		struct stats
		{
			u32 instances;  // crowns placed
			u32 models;     // distinct leaf geometries with leaves
			u32 triangles;
			u32 twins;      // triangles dropped as the back face of a card
			u32 nodes;      // BVH nodes over all models
			u32 textures;   // coverage pyramids
			u32 res_min, res_max; // their base resolution
			float coverage_mb;
			float memory_mb;
			float data_ms;  // geometry and textures read
			float bake_ms;

			u32 marches;
			u32 columns_walked;  // columns crossed that list a crown
			u32 boxes_tested;    // crown boxes clipped against the ray
			u32 boxes_entered;   // ...of those, actually crossed
			u32 node_tests;
			u32 triangle_tests;
			u32 hits;
		};

		// Only the per-march counters; everything else in stats is bake-time and survives.
		void reset_stats() const
		{
			m_stats.marches = 0;
			m_stats.columns_walked = 0;
			m_stats.boxes_tested = 0;
			m_stats.boxes_entered = 0;
			m_stats.node_tests = 0;
			m_stats.triangle_tests = 0;
			m_stats.hits = 0;
		}

		const stats& get_stats() const { return m_stats; }
#endif

	private:
		// 32 bytes. Nodes are stored depth first: an interior node's left child is the next node.
		struct bvh_node
		{
			Fvector bmin;
			u32 a;         // interior: right child; leaf: first triangle
			Fvector bmax;
			u32 count;     // triangles in a leaf; 0 = interior
		};

		struct crown_model
		{
			u32 coverage;  // into m_coverage
			u32 root;      // into m_nodes
		};

		struct crown
		{
			Fmatrix inv;   // world -> object
			Fbox box;      // world, slightly inflated so every column it touches lists it
			u32 model;
			float lod;     // -log2 of the instance's scale: a world width to object metres
			mutable u32 stamp; // the last march that tested it: a crown spans many columns
		};

		// Sight-line width along a march: eye + slope * t metres at t.
		struct cone_along
		{
			float eye;
			float slope;
			float near_plane;
		};

		bool build();

		// Appends the BVH over tris[lo, hi) of `ids` (triangle indices into the model, 3 vertices each)
		// and returns its root. tri_lod is per triangle of the model.
		u32 build_node(const canopy_model& cm, const xr_vector<float>& tri_lod, const xr_vector<Fvector>& centroid,
		               xr_vector<u32>& ids, u32 lo, u32 hi);

		// Optical depth of the cards of one crown pierced by from + dir * t, t in [t0, t1]; `hits`
		// receives how many were counted.
		float pierce(const crown& c, const Fvector& from, const Fvector& dir, float t0, float t1,
		             const cone_along& cone, u32& hits, xr_vector<canopy_hit>* rec) const;

		// Column directory, one entry per 2 m XZ column: offset into m_ids in the high 24 bits, crown
		// count in the low 8.
		static const u32 COL_COUNT_BITS = 8;
		static const u32 COL_COUNT_MAX = (1u << COL_COUNT_BITS) - 1;
		static const u32 COL_OFFSET_MAX = (1u << (32 - COL_COUNT_BITS)) - 1;

		u32 column(int gx, int gz) const { return m_col[u32(gz) * m_size_x + u32(gx)]; }

		xr_vector<Fvector> m_tris;   // object space, 3 per triangle, in BVH leaf order
		xr_vector<Fvector2> m_uvs;   // 3 per triangle, beside m_tris
		xr_vector<float> m_tri_lod;  // per triangle, log2(base texels per object metre)
		xr_vector<coverage_lod> m_coverage;
		xr_vector<bvh_node> m_nodes;
		xr_vector<crown_model> m_models;
		xr_vector<crown> m_crowns;
		xr_vector<u32> m_col;
		xr_vector<u32> m_ids;        // crown indices, grouped by column

		int m_min_sx = 0; // world slot index of column (0,0)
		int m_min_sz = 0;
		u32 m_size_x = 0;
		u32 m_size_z = 0;
		float m_y_lo = 0.f, m_y_hi = 0.f; // vertical extent of all crowns

		float m_finest_cell = 0.05f;
		float m_alpha_ref = 200.f / 255.f;

		bool m_ready = false;
		bool m_complete = false;
		bool m_overcounted = false;

		mutable u32 m_march_id = 0;

#ifdef PERCEPTION_DEBUG
		mutable stats m_stats;
#endif
	};
}
