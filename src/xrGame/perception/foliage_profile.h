#pragma once

#include "detail_data.h"
#include "dds_alpha.h"

namespace perception
{
	static const u32 TUFT_NR = 8; // radial bins over [0, radius]
	static const u32 TUFT_NY = 8; // height bins over [y_min, y_max]

	// Where a model's drawn opaque area is, averaged over the random yaw instances get: opaque area per
	// unit volume over distance from the model's Y axis and height, model space. Upright faces and flat
	// faces are kept apart because a ray's elevation projects them differently.
	struct tuft_density
	{
		float radius;       // largest vertex distance from the Y axis
		float y_min, y_max; // vertex Y range
		float upright[TUFT_NY][TUFT_NR];
		float flat[TUFT_NY][TUFT_NR];
		float opaque_area;  // m2 at the alpha cutoff
		float r50, r90;     // radii holding 50 % / 90 % of the opaque area
		float top95;        // height under which 95 % of it lies

		bool valid() const { return radius > 0.f && y_max > y_min && opaque_area > 0.f; }

		// Optical depth per metre at model-space q, for a ray whose horizontal and vertical direction
		// components are k_up (already times the upright projection factor) and k_flat.
		float extinction(const Fvector& q, float k_up, float k_flat) const;
	};

	struct foliage_model_profile
	{
		float height;  // metres, model-space bbox Y extent
		float max_placement_scale; // largest scale placement draws, before the detail height slider
		tuft_density density;
	};

	// Density has to come from the texture, not the mesh: detail models are a handful of crossed
	// quads and essentially all perceived density lives in the atlas alpha. alpha_ref is the shader's
	// alpha-test cutoff, 0..1.
	void build_foliage_profiles(const detail_data& dd, float alpha_ref, xr_vector<foliage_model_profile>& out);

	// Opens a texture by the engine's own alias order ($level$, $game_saves$, $game_textures$) so mod
	// overrides and level-local atlases resolve through the VFS for free. Caller closes.
	IReader* open_texture(LPCSTR name);
}
