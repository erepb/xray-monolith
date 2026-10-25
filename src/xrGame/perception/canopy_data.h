#pragma once

#include "../../build_config_defines.h"
#include "coverage_lod.h"

namespace perception
{
	// Distinct leaf geometry, in object space. Trees are instanced: Escape has 5289 leaf instances
	// over 200 of these, Red Forest 6228 over 136, so the triangles are read once per model and
	// transformed per instance.
	struct canopy_model
	{
		shared_str texture;
		u32 coverage;             // into canopy_data::coverage(); u32(-1) = no readable texture
		xr_vector<Fvector> verts; // 3 per triangle, each card once
		xr_vector<Fvector2> uvs;  // 3 per triangle, beside verts
	};

	// One placed tree crown.
	struct canopy_instance
	{
		Fmatrix xform;
		u32 model;
	};

	// Read-only view over the leaf geometry in `level` + `level.geom`.
	//
	// Parsed here rather than requested from the renderer, which is the opposite of the choice grass
	// makes (foliage_oracle calls IRender_interface::resolve_detail_slot) and for the opposite
	// reasons: canopy geometry carries no runtime sliders - the tree wave constants animate leaves in
	// the vertex shader without moving the mean geometry - and CRender::LoadBuffers frees its CPU copy
	// of the vertex and index data after upload (r4_loader.cpp:238,270), so the renderer cannot supply
	// it. See CANOPY-DESIGN 2.
	class canopy_data
	{
	public:
		canopy_data();
		~canopy_data();

		// finest_cell: metres of card the base level of each coverage pyramid resolves.
		// alpha_ref: 0..1, the cutoff for textures whose alpha is not one bit.
		bool load(float finest_cell, float alpha_ref);
		void unload();

		const xr_vector<canopy_model>& models() const { return m_models; }
		const xr_vector<canopy_instance>& instances() const { return m_instances; }

		// One per distinct leaf texture. Writable so the caller can take them over.
		xr_vector<coverage_lod>& coverage() { return m_coverage; }

		// Triangles dropped because another of the same model has the same three positions: leaf cards
		// are stored as a front and a back face.
		u32 twins() const { return m_twins; }

		// Leaf-shader visuals that were NOT a tree type, and so contributed nothing. Zero on both
		// levels measured, but that is two levels: a non-zero count here is a coverage gap in this
		// parser rather than a level with no canopy, which is why it is reported rather than ignored.
		u32 non_tree_leaf() const { return m_non_tree_leaf; }

		// Distinct textures that could not be read. The models using them are NOT dropped: they keep
		// no coverage and the oracle skips them, so a non-zero count means crowns that contribute nothing.
		u32 rejected() const { return m_rejected; }

		// Progressive-mesh tree visuals whose sliding window could not be resolved (chunk missing, id
		// out of range, or an empty entry). Their model falls back to the full index range, which sums
		// every LOD level, so the leaf area they contribute is over-counted.
		u32 unresolved_swi() const { return m_unresolved_swi; }

		// Models with triangles but no texture name in the shader table. They have no alpha to read
		// and so contribute nothing.
		u32 no_texture() const { return m_no_texture; }

		// Leaf geometry that was found but could not be used (truncated or compressed visuals chunk,
		// missing geometry or tree-definition chunk, out-of-range buffer references). The field does not
		// account for it. An event count, not a unit count: it is incremented per instance in some
		// places, per model in others, and once for a truncated walk that can drop many visuals, so
		// only whether it is zero matters.
		u32 skipped() const { return m_skipped; }

	private:
		struct vb_desc
		{
			const u8* data;
			u32 stride;
			u32 count;
			u32 pos_offset;
			u32 tc_offset;
			u8 tc_type;  // DECLTYPE of TEXCOORD0; DECLTYPE_UNUSED when absent
			bool usable; // a float3 POSITION element was found
		};

		struct ib_desc
		{
			const u16* data;
			u32 count;
		};

		// Only the maximum-detail window is needed. FTreeVisual_PM::Render maps LOD 1.0 to index 0,
		// and the index range holds every level - taking iCount instead would sum them all.
		// A malformed or empty entry is kept as {0, 0} so the table stays index-aligned with the file.
		struct swi_desc
		{
			u32 offset;
			u16 tris;
		};

		bool read_geom();
		bool read_shaders(IReader* level);
		bool read_visuals(IReader* level);
		void read_visual(IReader* v);
		void decode_models();
		void drop_twins(canopy_model& cm);
		void resolve_coverage(float finest_cell, float alpha_ref);

		// Closes level.geom and frees the descriptors that point into it. Nothing after decode_models()
		// needs them, and the buffer is ~100 MB on Escape.
		void release_geom();

		IReader* m_geom;
		xr_vector<vb_desc> m_vb;
		xr_vector<ib_desc> m_ib;
		xr_vector<swi_desc> m_swi;

		xr_vector<shared_str> m_shader; // by shader_id, as the level stores them
		xr_vector<shared_str> m_texture;

		// Geometry range a model was built from, kept only to deduplicate instances onto models.
		struct model_key
		{
			u32 vb, vbase, ib, ibase, icount, swi, shader_id;
			bool operator==(const model_key& o) const
			{
				return vb == o.vb && vbase == o.vbase && ib == o.ib && ibase == o.ibase
					&& icount == o.icount && swi == o.swi && shader_id == o.shader_id;
			}
		};

		xr_vector<model_key> m_keys;
		xr_vector<canopy_model> m_models;
		xr_vector<canopy_instance> m_instances;
		xr_vector<coverage_lod> m_coverage;

		u32 m_non_tree_leaf;
		u32 m_rejected;
		u32 m_unresolved_swi;
		u32 m_no_texture;
		u32 m_skipped;
		u32 m_twins;
	};
}
