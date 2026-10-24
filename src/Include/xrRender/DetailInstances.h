#pragma once

// One placed detail object, as the renderer actually placed it.
//
// Crosses the render/AI boundary the same way atmo_lut does: the renderer owns the computation
// because only it has the placement code, the CDB terrain and the two user sliders that feed it, and
// the AI receives plain data. Nothing here is derived - it is what the player is looking at.
//
// Yaw is deliberately absent. The occlusion proxy on the AI side is a vertical cylinder, which is
// rotationally symmetric, so neither the random yaw nor ps_ssfx_terrain_grass_align can change the
// answer and carrying them would only invite someone to depend on them.
struct detail_instance
{
	Fvector pos;   // world, base of the tuft
	float scale;   // model scale, ps_current_detail_height already folded in
	u16 model;     // index into the level's detail model table (DetailSlot::r_id)
};
