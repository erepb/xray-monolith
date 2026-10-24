#include "stdafx.h"
#pragma hdrstop

#include "DetailManager.h"
#include "cl_intersect.h"

#ifdef _EDITOR
#	include "scene.h"
#	include "sceneobject.h"
#	include "../utils/ETools/ETools.h"
#endif

//--------------------------------------------------- Decompression
IC float Interpolate(float* base, u32 x, u32 y, u32 size)
{
	float f = float(size);
	float fx = float(x) / f;
	float ifx = 1.f - fx;
	float fy = float(y) / f;
	float ify = 1.f - fy;

	float c01 = base[0] * ifx + base[1] * fx;
	float c23 = base[2] * ifx + base[3] * fx;

	float c02 = base[0] * ify + base[2] * fy;
	float c13 = base[1] * ify + base[3] * fy;

	float cx = ify * c01 + fy * c23;
	float cy = ifx * c02 + fx * c13;
	return (cx + cy) / 2;
}

IC bool InterpolateAndDither(float* alpha255, u32 x, u32 y, u32 sx, u32 sy, u32 size, int dither[16][16])
{
	clamp(x, (u32)0, size - 1);
	clamp(y, (u32)0, size - 1);
	int c = iFloor(Interpolate(alpha255, x, y, size) + .5f);
	clamp(c, 0, 255);

	u32 row = (y + sy) % 16;
	u32 col = (x + sx) % 16;
	return c > dither[col][row];
}

#ifndef _EDITOR
#ifdef	DEBUG
//#include "../../Include/xrRender/DebugRender.h"
#include "dxDebugRender.h"
static void draw_obb		( const Fmatrix &matrix, const u32 &color )
{
	Fvector							aabb[8];
	matrix.transform_tiny			(aabb[0],Fvector().set( -1, -1, -1)); // 0
	matrix.transform_tiny			(aabb[1],Fvector().set( -1, +1, -1)); // 1
	matrix.transform_tiny			(aabb[2],Fvector().set( +1, +1, -1)); // 2
	matrix.transform_tiny			(aabb[3],Fvector().set( +1, -1, -1)); // 3
	matrix.transform_tiny			(aabb[4],Fvector().set( -1, -1, +1)); // 4
	matrix.transform_tiny			(aabb[5],Fvector().set( -1, +1, +1)); // 5
	matrix.transform_tiny			(aabb[6],Fvector().set( +1, +1, +1)); // 6
	matrix.transform_tiny			(aabb[7],Fvector().set( +1, -1, +1)); // 7

	u16								aabb_id[12*2] = {
		0,1,  1,2,  2,3,  3,0,  4,5,  5,6,  6,7,  7,4,  1,5,  2,6,  3,7,  0,4
	};

	rdebug_render->add_lines						(aabb, sizeof(aabb)/sizeof(Fvector), &aabb_id[0], sizeof(aabb_id)/(2*sizeof(u16)), color);
}

bool det_render_debug = false;
#endif
#endif

#include "../../xrEngine/gamemtllib.h"

static inline u32 hash2(u32 x, u32 y)
{
	u64 a = ((u64)x << 32) | (u64)y;
	u64 r = 9199940308585234877ull * a + 9199940308585234877ull;
	return (u32)(r >> 32);
}

//#define		DBG_SWITCHOFF_RANDOMIZE

// The single placement routine. Everything that decides WHERE a detail object stands lives here and
// nowhere else - the render cache and the AI-side resolver are both callers, so neither can drift
// from what the player is actually looking at.
void CDetailManager::place_slot(int sx, int sz, IPlacer& out)
{
	DetailSlot& DS = QueryDB(sx, sz);
	if ((DS.id0 == DetailSlot::ID_Empty) && (DS.id1 == DetailSlot::ID_Empty) &&
		(DS.id2 == DetailSlot::ID_Empty) && (DS.id3 == DetailSlot::ID_Empty))
		return;

	// Same box cache_CD builds (DetailManager_CACHE.cpp:59-61), rebuilt from the grid coordinate so
	// this works for any slot in the level and not only the ones the camera cache happens to hold.
	Fbox slot_box;
	slot_box.min.set(sx * dm_slot_size, DS.r_ybase(), sz * dm_slot_size);
	slot_box.max.set(slot_box.min.x + dm_slot_size, DS.r_ybase() + DS.r_yheight(),
	                 slot_box.min.z + dm_slot_size);
	slot_box.grow(EPS_L);

	// Select polygons
	Fvector bC, bD;
	slot_box.get_CD(bC, bD);

#ifdef _EDITOR
	ETOOLS::box_options	(CDB::OPT_FULL_TEST);
	// Select polygons
	SBoxPickInfoVec		pinf;
    Scene->BoxPickObjects(slot_box,pinf,GetSnapList());
	u32	triCount		= pinf.size();
#else
	xrc.box_options(CDB::OPT_FULL_TEST);
	xrc.box_query(g_pGameLevel->ObjectSpace.GetStaticModel(), bC, bD);
	u32 triCount = xrc.r_count();
	CDB::TRI* tris = g_pGameLevel->ObjectSpace.GetStaticTris();
	Fvector* verts = g_pGameLevel->ObjectSpace.GetStaticVerts();
#endif

	if (0 == triCount) return;

	// Build shading table
	float alpha255 [dm_obj_in_slot][4];
	for (int i = 0; i < dm_obj_in_slot; i++)
	{
		alpha255[i][0] = 255.f * float(DS.palette[i].a0) / 15.f;
		alpha255[i][1] = 255.f * float(DS.palette[i].a1) / 15.f;
		alpha255[i][2] = 255.f * float(DS.palette[i].a2) / 15.f;
		alpha255[i][3] = 255.f * float(DS.palette[i].a3) / 15.f;
	}

	// Prepare to selection
	float density = ps_r__Detail_density;
	float jitter = density / 1.7f;
	u32 d_size = iCeil(dm_slot_size / density);
	svector<int, dm_obj_in_slot> selected;

	u32 p_rnd = hash2((u32)sx, (u32)sz);
	CRandom r_selection(0x12071980 ^ p_rnd);
	CRandom r_jitter(0x12071980 ^ p_rnd);
	CRandom r_yaw(0x12071980 ^ p_rnd);
	CRandom r_scale(0x12071980 ^ p_rnd);

	// Decompressing itself
	for (u32 z = 0; z <= d_size; z++)
	{
		for (u32 x = 0; x <= d_size; x++)
		{
			// shift
#ifndef		DBG_SWITCHOFF_RANDOMIZE
			u32 shift_x = r_jitter.randI(16);
			u32 shift_z = r_jitter.randI(16);
#else
			u32 shift_x =  8;
			u32 shift_z =  8;
#endif
			// Iterpolate and dither palette
			selected.clear();

#ifndef		DBG_SWITCHOFF_RANDOMIZE
			if ((DS.id0 != DetailSlot::ID_Empty) && InterpolateAndDither(
				alpha255[0], x, z, shift_x, shift_z, d_size, dither))
				selected.push_back(0);
			if ((DS.id1 != DetailSlot::ID_Empty) && InterpolateAndDither(
				alpha255[1], x, z, shift_x, shift_z, d_size, dither))
				selected.push_back(1);
			if ((DS.id2 != DetailSlot::ID_Empty) && InterpolateAndDither(
				alpha255[2], x, z, shift_x, shift_z, d_size, dither))
				selected.push_back(2);
			if ((DS.id3 != DetailSlot::ID_Empty) && InterpolateAndDither(
				alpha255[3], x, z, shift_x, shift_z, d_size, dither))
				selected.push_back(3);
#else
			if ((DS.id0!=DetailSlot::ID_Empty))	selected.push_back(0);
			if ((DS.id1!=DetailSlot::ID_Empty))	selected.push_back(1);
			if ((DS.id2!=DetailSlot::ID_Empty))	selected.push_back(2);
			if ((DS.id3!=DetailSlot::ID_Empty))	selected.push_back(3);
#endif

			// Select
			if (selected.empty()) continue;
#ifndef		DBG_SWITCHOFF_RANDOMIZE
			u32 index;
			if (selected.size() == 1) index = selected[0];
			else index = selected[r_selection.randI(selected.size())];
#else
			u32 index = selected[0];
#endif

			CDetail* Dobj = objects[DS.r_id(index)];
			float i_scale;
			Fmatrix i_rot;

			// Position (XZ)
			float rx = (float(x) / float(d_size)) * dm_slot_size + slot_box.min.x;
			float rz = (float(z) / float(d_size)) * dm_slot_size + slot_box.min.z;
			Fvector Item_P;

#ifndef		DBG_SWITCHOFF_RANDOMIZE
			Item_P.set(rx + r_jitter.randFs(jitter), slot_box.max.y, rz + r_jitter.randFs(jitter));
#else
			Item_P.set	(rx , D.vis.box.max.y, rz );
#endif

			// Position (Y)
			float y = slot_box.min.y - 5;
			Fvector dir;
			dir.set(0, -1, 0);
			Fvector3 terrain_normal;

			float r_u, r_v, r_range;
			for (u32 tid = 0; tid < triCount; tid++)
			{
#ifdef _EDITOR
				Fvector verts[3];
				SBoxPickInfo& I=pinf[tid];
				for (int k=0; k<(int)I.inf.size(); k++){
					VERIFY(I.s_obj);
RDEVICE.Statistic->TEST0.Begin	();
					I.e_obj->GetFaceWorld(I.s_obj->_Transform(),I.e_mesh,I.inf[k].id,verts);
RDEVICE.Statistic->TEST0.End		();
					if (CDB::TestRayTri(Item_P,dir,verts,r_u,r_v,r_range,TRUE))
					{
						if (r_range>=0)	{
							float y_test	= Item_P.y - r_range;
							if (y_test>y)	y = y_test;
						}
					}
				}
#else
				CDB::TRI& T = tris[xrc.r_begin()[tid].id];
				SGameMtl* mtl = GMLib.GetMaterialByIdx(T.material);
				if (mtl->Flags.test(SGameMtl::flPassable))
					continue;

				Fvector Tv[3] = {verts[T.verts[0]], verts[T.verts[1]], verts[T.verts[2]]};
				if (CDB::TestRayTri(Item_P, dir, Tv, r_u, r_v, r_range,TRUE))
				{
					if (r_range >= 0)
					{
						float y_test = Item_P.y - r_range;
						if (y_test > y) y = y_test;
						terrain_normal.mknormal(Tv[0], Tv[1], Tv[2]);
					}
				}
#endif
			}

			// Slope Limit
			float DotP = terrain_normal.dotproduct(dir);
			if (DotP > -(1.0f - r_scale.randF(ps_ssfx_terrain_grass_slope * 0.8f, ps_ssfx_terrain_grass_slope))) continue;

			if (y < slot_box.min.y) continue;
			Item_P.y = y;


			// Angles and scale
#ifndef		DBG_SWITCHOFF_RANDOMIZE
			i_scale = r_scale.randF(Dobj->m_fMinScale * 0.5f, Dobj->m_fMaxScale * 0.9f);
#else
			i_scale	= (Dobj->m_fMinScale*0.5f+Dobj->m_fMaxScale*0.9f)/2;
#endif
			i_scale *= ps_current_detail_height;
#ifndef		DBG_SWITCHOFF_RANDOMIZE
			i_rot.rotateY(r_yaw.randF(0, PI_MUL_2));
#else
			i_rot.rotateY				(0);
#endif

			// Terrain Alignment
			if (ps_ssfx_terrain_grass_align > 0)
			{
				// Current matrix
				Fmatrix CurrMatrix = i_rot;

				// Align to terrain
				i_rot.j.set(terrain_normal);
				Fvector::generate_orthonormal_basis(i_rot.j, i_rot.i, i_rot.k);

				// Apply random rotation from old matrix
				i_rot.mulB_43(CurrMatrix);
			}

			i_rot.translate_over(Item_P);

			out.place(index, DS.r_id(index), Item_P, i_scale, i_rot, terrain_normal);
		}
	}
}

namespace
{
	// The render cache's half of placement: everything downstream of "an instance stands here" -
	// pooling, lighting, wave sorting and the tightened bounds.
	class render_placer : public CDetailManager::IPlacer
	{
	public:
		CDetailManager* dm;
		CDetailManager::Slot* slot;
		DetailSlot* ds;
		Fbox bounds;

		render_placer() : dm(0), slot(0), ds(0) { bounds.invalidate(); }

		virtual void place(u32 slot_slot, u32 model_id, const Fvector& pos, float scale,
		                   const Fmatrix& rot, const Fvector& terrain_normal)
		{
			CDetail* Dobj = dm->objects[model_id];
			CDetailManager::SlotItem* ItemP = dm->poolSI.create();
			CDetailManager::SlotItem& Item = *ItemP;

			Item.scale = scale;
			Item.mRotY = rot;
			Item.normal = terrain_normal; // fed to the grass shader later

			Fmatrix mScale, mXform;
			Fbox ItemBB;
			mScale.scale(scale, scale, scale);
			mXform.mul_43(Item.mRotY, mScale);
			ItemBB.xform(Dobj->bv_bb, mXform);
			bounds.merge(ItemBB);

#ifndef _EDITOR
#ifdef		DEBUG
			if (det_render_debug)
				draw_obb(mXform, color_rgba(255, 0, 0, 255));
#endif
#endif

#if RENDER==R_R1
			Item.c_rgb.x = ds->r_qclr(ds->c_r, 15);
			Item.c_rgb.y = ds->r_qclr(ds->c_g, 15);
			Item.c_rgb.z = ds->r_qclr(ds->c_b, 15);
#endif
			Item.c_hemi = ds->r_qclr(ds->c_hemi, 15);
			Item.c_sun = ds->r_qclr(ds->c_dir, 15);

			// Vis-sorting
#ifndef		DBG_SWITCHOFF_RANDOMIZE
			if (!dm->UseVS())
			{
				// Always still on CPU pipe
				Item.vis_ID = 0;
			}
			else
			{
				if (Dobj->m_Flags.is(DO_NO_WAVING)) Item.vis_ID = 0;
				else
				{
					if (::Random.randI(0, 3) == 0) Item.vis_ID = 2; // Second wave
					else Item.vis_ID = 1; // First wave
				}
			}
#else
			Item.vis_ID = 0;
#endif
			slot->G[slot_slot].items.push_back(ItemP);
		}
	};
}

void CDetailManager::cache_Decompress(Slot* S)
{
	VERIFY(S);
	Slot& D = *S;
	D.type = stReady;
	if (D.empty) return;

	render_placer rp;
	rp.dm = this;
	rp.slot = &D;
	rp.ds = &QueryDB(D.sx, D.sz);

	place_slot(D.sx, D.sz, rp);

	// Update bounds to more tight and real ones
	D.vis.clear();
	D.vis.box.set(rp.bounds);
	D.vis.box.getsphere(D.vis.sphere.P, D.vis.sphere.R);
}

namespace
{
	// Drops yaw and the terrain normal: the AI-side proxy is a vertical cylinder, so neither can
	// change the answer and carrying them would only invite a dependency on them.
	class collect_placer : public CDetailManager::IPlacer
	{
	public:
		xr_vector<detail_instance>* out;

		virtual void place(u32 slot_slot, u32 model_id, const Fvector& pos, float scale,
		                   const Fmatrix& rot, const Fvector& terrain_normal)
		{
			detail_instance i;
			i.pos = pos;
			i.scale = scale;
			i.model = u16(model_id);
			out->push_back(i);
		}
	};
}

void CDetailManager::collect_slot(int sx, int sz, xr_vector<detail_instance>& out)
{
	collect_placer cp;
	cp.out = &out;
	place_slot(sx, sz, cp);
}
