#include "stdafx.h"
#include "bone_shape.h"
#include "../../xrEngine/bone.h"

namespace perception
{
	float bone_shape_area(const SBoneShape& shape)
	{
		float surface_area = flt_max;
		switch (shape.type)
		{
		case SBoneShape::stBox:
			{
				Fvector const& half_size = shape.box.m_halfsize;
				surface_area = 2.f * (half_size.x * (half_size.y + half_size.z) + half_size.y * half_size.z);
				break;
			}
		case SBoneShape::stSphere:
			{
				surface_area = 4.f * PI * _sqr(shape.sphere.R);
				break;
			}
		case SBoneShape::stCylinder:
			{
				surface_area = 2.f * PI * shape.cylinder.m_radius * (shape.cylinder.m_radius + shape.cylinder.m_height);
				break;
			}
		default: NODEFAULT;
		}
		return surface_area;
	}

	Fvector random_point_on_bone_shape(const SBoneShape& shape, CRandom& rnd)
	{
		Fvector result = Fvector().set(flt_max, flt_max, flt_max);
		switch (shape.type)
		{
		case SBoneShape::stBox:
			{
				Fmatrix transform;
				shape.box.xform_full(transform);

				Fvector direction;
				u32 random_value = rnd.randI(6);
				Fvector random = {(random_value & 1) ? -1.f : 1.f, rnd.randF(2.f) - 1.f, rnd.randF(2.f) - 1.f};
				random.normalize();
				if (random_value < 2)
					direction = Fvector().set(random.x, random.y, random.z);
				else if (random_value < 4)
					direction = Fvector().set(random.z, random.x, random.y);
				else
					direction = Fvector().set(random.y, random.z, random.x);

				transform.transform_tiny(result, direction);
				break;
			}
		case SBoneShape::stSphere:
			{
				result.random_dir(rnd).mul(shape.sphere.R).add(shape.sphere.P);
				break;
			}
		case SBoneShape::stCylinder:
			{
				float const total_square = (shape.cylinder.m_height + shape.cylinder.m_radius); // *2*PI*c_cylinder.m_radius
				float const random_value = rnd.randF(total_square);
				float const angle = rnd.randF(2.f * PI);

				float const x = shape.cylinder.m_direction.x;
				float const y = shape.cylinder.m_direction.y;
				float const z = shape.cylinder.m_direction.z;
				Fvector normal = Fvector().set(y - z, z - x, x - y);

				Fmatrix rotation = Fmatrix().rotation(shape.cylinder.m_direction, normal);
				Fmatrix rotation_y = Fmatrix().rotateY(angle);
				Fmatrix const transform = Fmatrix().mul_43(rotation, rotation_y);
				transform.transform_dir(normal, Fvector().set(0.f, 0.f, 1.f));

				float height, radius;
				if (random_value < shape.cylinder.m_height)
				{
					height = random_value - shape.cylinder.m_height / 2.f;
					radius = shape.cylinder.m_radius;
				}
				else
				{
					float const normalized_value = random_value - shape.cylinder.m_height;
					height = shape.cylinder.m_height / 2.f * ((normalized_value < shape.cylinder.m_radius / 2.f)
						                                          ? -1.f
						                                          : 1.f);
					radius = height > 0.f ? normalized_value - shape.cylinder.m_radius / 2.f : normalized_value;
				}

				normal.mul(radius);
				result.mul(shape.cylinder.m_direction, height);
				result.add(normal);
				result.add(shape.cylinder.m_center);
				break;
			}
		default: NODEFAULT;
		}
		return result;
	}
}
