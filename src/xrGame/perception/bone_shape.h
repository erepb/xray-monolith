#pragma once

struct SBoneShape;

namespace perception
{
	// One implementation of "a point on the body", shared by v1 (CEntityAlive) and the v2 sampler so
	// the two sample the same population with the same weights.
	float bone_shape_area(const SBoneShape& shape);

	// Bone-local point on the shape's surface. Draw order on `rnd` is part of the contract: v1 passes
	// ::Random and must keep consuming it exactly as before.
	Fvector random_point_on_bone_shape(const SBoneShape& shape, CRandom& rnd);
}
