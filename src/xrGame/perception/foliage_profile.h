#pragma once

#include "detail_data.h"
#include "dds_alpha.h"

namespace perception
{
	struct foliage_model_profile
	{
		float height;  // metres, model-space bbox Y extent
		float radius;  // metres, half the larger model-space bbox XZ extent - the tuft's footprint
		float opacity; // 0..1, area-weighted mean texture alpha over the model's faces
		float gap;     // 0..1 share of the model's atlas footprint that is effectively a hole
		float area;    // summed triangle area, model space
	};

	// Opacity has to come from the texture, not the mesh: detail models are a handful of crossed
	// quads and essentially all perceived density lives in the atlas alpha. Per-face UV restriction
	// is mandatory because vanilla points every model at one shared atlas.
	void build_foliage_profiles(const detail_data& dd, xr_vector<foliage_model_profile>& out);

	// Opens a texture by the engine's own alias order ($level$, $game_saves$, $game_textures$) so mod
	// overrides and level-local atlases resolve through the VFS for free. Caller closes.
	IReader* open_texture(LPCSTR name);

	// Mean alpha of a texture. uv may be invalid to sample the whole image. out_info, if non-null,
	// receives the probed dimensions/format. out_gap, if non-null, receives the gap fraction (see
	// dds_mean_alpha).
	bool texture_mean_alpha(LPCSTR name, const uv_rect& uv, float& out_alpha, dds_info* out_info = 0,
	                        float* out_gap = 0);
}
