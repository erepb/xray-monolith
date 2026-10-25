#include "stdafx.h"
#include <algorithm>
#include "canopy_data.h"
#include "detail_data.h"
#include "foliage_profile.h"
#include "perception_debug.h"
#include "../../xrEngine/fmesh.h"
#include "../../xrEngine/xrLevel.h"

namespace perception
{
	// D3DDECLTYPE -> bytes. The vertex layout is data-driven (CRender::LoadBuffers reads a
	// D3DVERTEXELEMENT9 declaration per buffer), so these sizes decide the stride. Getting SHORT2 or
	// SHORT4 wrong does not fail loudly - it drifts the stride and silently corrupts every buffer
	// after the first.
	static const u32 DECLTYPE_BYTES[] = {
		4,  // FLOAT1
		8,  // FLOAT2
		12, // FLOAT3
		16, // FLOAT4
		4,  // D3DCOLOR
		4,  // UBYTE4
		4,  // SHORT2
		8,  // SHORT4
		4,  // UBYTE4N
		4,  // SHORT2N
		8,  // SHORT4N
		4,  // USHORT2N
		8,  // USHORT4N
		4,  // UDEC3
		4,  // DEC3N
		4,  // FLOAT16_2
		8,  // FLOAT16_4
		0   // UNUSED
	};
	static const u32 DECLTYPE_COUNT = sizeof(DECLTYPE_BYTES) / sizeof(DECLTYPE_BYTES[0]);
	static const u32 DECLTYPE_FLOAT2 = 1;
	static const u32 DECLTYPE_FLOAT3 = 2;
	static const u32 DECLTYPE_SHORT2 = 6;
	static const u32 DECLTYPE_SHORT4 = 7;
	static const u8 DECLTYPE_UNUSED = 17;
	static const u32 DECLUSAGE_POSITION = 0;
	static const u32 DECLUSAGE_TEXCOORD = 5;
	static const u16 DECL_STREAM_END = 0xFF;
	static const u32 DECL_ELEMENT_BYTES = 8;

	// Tree texture coordinates stored as shorts are scaled by FTreeVisual_quant (FTreeVisual.h).
	static const float TREE_TC_QUANT = 32768.f / 16.f;

	// Positions that agree to this many metres are the same vertex for twin detection.
	static const double TWIN_QUANT = 1e-4;

	static IC Fvector2 read_uv(const u8* v, u8 type)
	{
		Fvector2 r;
		if (DECLTYPE_FLOAT2 == type)
			r = *(const Fvector2*)v;
		else
			r.set(float(*(const s16*)v) / TREE_TC_QUANT, float(*(const s16*)(v + 2)) / TREE_TC_QUANT);
		return r;
	}

	static IC u32 remaining_of(IReader* r) { const int e = r->elapsed(); return e > 0 ? u32(e) : 0; }

	// find_chunk plus a minimum-size check, returning the data directly.
	//
	// Not IReader::r_chunk_safe: that asserts the chunk size equals the destination size exactly, and
	// OGF_TREEDEF2 is 104 bytes (a matrix plus two colour triples) when only the leading matrix is
	// wanted - reading it that way would trip the assert on every tree. find_chunk's declared size is
	// also unvalidated in release, so it is bounded against the reader here.
	static const u8* chunk_at(IReader* r, u32 id, u32 need)
	{
		const u32 sz = r->find_chunk(id);
		if (sz < need || sz > remaining_of(r))
			return 0;
		return (const u8*)r->pointer();
	}

	static bool is_leaf_shader(LPCSTR s)
	{
		// flora\trunk_wave is bark - opaque, and present on the very same tree visuals as the canopy.
		return s && 0 == strncmp(s, "flora", 5) && !strstr(s, "trunk");
	}

	canopy_data::canopy_data()
		: m_geom(0), m_non_tree_leaf(0), m_rejected(0), m_unresolved_swi(0),
		  m_no_texture(0), m_skipped(0), m_twins(0)
	{
	}

	canopy_data::~canopy_data()
	{
		unload();
	}

	void canopy_data::release_geom()
	{
		if (m_geom)
		{
			FS.r_close(m_geom);
			m_geom = 0;
		}
		xr_vector<vb_desc>().swap(m_vb);
		xr_vector<ib_desc>().swap(m_ib);
		xr_vector<swi_desc>().swap(m_swi);
	}

	void canopy_data::unload()
	{
		release_geom();
		xr_vector<shared_str>().swap(m_shader);
		xr_vector<shared_str>().swap(m_texture);
		xr_vector<model_key>().swap(m_keys);
		xr_vector<canopy_model>().swap(m_models);
		xr_vector<canopy_instance>().swap(m_instances);
		xr_vector<coverage_lod>().swap(m_coverage);
		m_non_tree_leaf = 0;
		m_rejected = 0;
		m_unresolved_swi = 0;
		m_no_texture = 0;
		m_skipped = 0;
		m_twins = 0;
	}

	bool canopy_data::load(float finest_cell, float alpha_ref)
	{
		unload();

		if (!FS.exist("$level$", "level") || !FS.exist("$level$", "level.geom"))
		{
			PDBG_LOG("~ [perception] canopy: level or level.geom absent - no canopy field");
			return false;
		}

		// level.geom is held open until decode_models() has copied the leaf triangles out, because the
		// vertex and index descriptors below point into its buffer rather than copying it. On Escape
		// that is ~100 MB, released by decode_models() before the texture reads begin.
		string_path fn;
		FS.update_path(fn, "$level$", "level.geom");
		m_geom = FS.r_open(fn);
		if (!m_geom)
		{
			Msg("! [perception] canopy: failed to open level.geom");
			return false;
		}
		if (!read_geom())
			return false;

		FS.update_path(fn, "$level$", "level");
		IReader* level = FS.r_open(fn);
		if (!level)
		{
			Msg("! [perception] canopy: failed to open level file");
			return false;
		}

		const bool ok = read_shaders(level) && read_visuals(level);
		FS.r_close(level);
		if (!ok)
			return false;

		decode_models();
		resolve_coverage(finest_cell, alpha_ref);
		return !m_instances.empty();
	}

	bool canopy_data::read_geom()
	{
		{
			IReader* vb = open_chunk_checked(m_geom, fsL_VB);
			if (!vb)
			{
				Msg("! [perception] canopy: level.geom has no vertex chunk");
				return false;
			}
			if (vb->elapsed() < 4)
			{
				vb->close();
				return false;
			}
			const u32 n = vb->r_u32();
			if (u64(n) * 12 > u64(remaining_of(vb)))
			{
				Msg("! [perception] canopy: VB count %u implausible for %u bytes", n, remaining_of(vb));
				vb->close();
				return false;
			}
			m_vb.reserve(n);
			for (u32 i = 0; i < n; ++i)
			{
				vb_desc d;
				d.data = 0;
				d.stride = 0;
				d.count = 0;
				d.pos_offset = 0;
				d.tc_offset = 0;
				d.tc_type = DECLTYPE_UNUSED;
				d.usable = false;

				// The declaration runs to an element whose stream is 0xFF, inclusive.
				u32 stride = 0;
				bool terminated = false;
				while (remaining_of(vb) >= DECL_ELEMENT_BYTES)
				{
					const u8* e = (const u8*)vb->pointer();
					const u16 stream = *(const u16*)(e + 0);
					const u16 offset = *(const u16*)(e + 2);
					const u8 type = e[4];
					const u8 usage = e[6];
					const u8 usage_index = e[7];
					vb->advance(DECL_ELEMENT_BYTES);

					if (DECL_STREAM_END == stream)
					{
						terminated = true;
						break;
					}
					if (type >= DECLTYPE_COUNT)
					{
						Msg("! [perception] canopy: VB %u declares unknown element type %u", i, type);
						terminated = false;
						break;
					}
					stride = _max(stride, u32(offset) + DECLTYPE_BYTES[type]);
					if (DECLUSAGE_POSITION == usage && 0 == usage_index && DECLTYPE_FLOAT3 == type)
					{
						d.pos_offset = offset;
						d.usable = true;
					}
					if (DECLUSAGE_TEXCOORD == usage && 0 == usage_index
						&& (DECLTYPE_FLOAT2 == type || DECLTYPE_SHORT2 == type || DECLTYPE_SHORT4 == type))
					{
						d.tc_offset = offset;
						d.tc_type = type;
					}
				}
				if (!terminated || !stride || remaining_of(vb) < 4)
				{
					Msg("! [perception] canopy: VB %u declaration malformed - canopy field abandoned", i);
					vb->close();
					return false;
				}

				d.stride = stride;
				d.count = vb->r_u32();
				const u64 bytes = u64(d.count) * d.stride;
				if (bytes > u64(remaining_of(vb)))
				{
					Msg("! [perception] canopy: VB %u claims %u verts x %u B beyond the chunk", i,
					    d.count, d.stride);
					vb->close();
					return false;
				}
				d.data = (const u8*)vb->pointer();
				vb->advance(int(bytes));
				m_vb.push_back(d);
			}
			vb->close();
		}

		{
			IReader* ib = open_chunk_checked(m_geom, fsL_IB);
			if (!ib || ib->elapsed() < 4)
			{
				Msg("! [perception] canopy: level.geom has no index chunk");
				if (ib)
					ib->close();
				return false;
			}
			const u32 n = ib->r_u32();
			if (u64(n) * 4 > u64(remaining_of(ib)))
			{
				Msg("! [perception] canopy: IB count %u implausible for %u bytes", n, remaining_of(ib));
				ib->close();
				return false;
			}
			m_ib.reserve(n);
			for (u32 i = 0; i < n; ++i)
			{
				if (remaining_of(ib) < 4)
				{
					ib->close();
					return false;
				}
				ib_desc d;
				d.count = ib->r_u32();
				const u64 bytes = u64(d.count) * 2;
				if (bytes > u64(remaining_of(ib)))
				{
					Msg("! [perception] canopy: IB %u claims %u indices beyond the chunk", i, d.count);
					ib->close();
					return false;
				}
				d.data = (const u16*)ib->pointer();
				ib->advance(int(bytes));
				m_ib.push_back(d);
			}
			ib->close();
		}

		// Absent on levels with no progressive meshes, which is not an error.
		IReader* sw = open_chunk_checked(m_geom, fsL_SWIS);
		if (sw)
		{
			if (sw->elapsed() >= 4)
			{
				const u32 n = sw->r_u32();
				if (u64(n) * 20 > u64(remaining_of(sw)))
				{
					Msg("! [perception] canopy: SWI count %u implausible for %u bytes", n,
						remaining_of(sw));
					sw->close();
					return false;
				}
				m_swi.reserve(n);

				swi_desc none;
				none.offset = 0;
				none.tris = 0;

				// Every entry yields exactly one slot, usable or not: a table that shrinks would shift
				// every later index onto the wrong window.
				for (u32 i = 0; i < n; ++i)
				{
					if (remaining_of(sw) < 20)
					{
						m_swi.push_back(none);
						continue;
					}
					sw->advance(16); // reserved[4]
					const u32 count = sw->r_u32();
					if (!count)
					{
						m_swi.push_back(none);
						continue;
					}
					const u64 bytes = u64(count) * 8; // FSlideWindow: u32 offset, u16 tris, u16 verts
					if (bytes > u64(remaining_of(sw)))
					{
						// The rest of the table cannot be located; the remaining iterations pad it.
						m_swi.push_back(none);
						sw->advance(int(remaining_of(sw)));
						continue;
					}

					swi_desc d;
					d.offset = sw->r_u32();
					d.tris = sw->r_u16();
					sw->advance(2);
					sw->advance(int(bytes) - 8);
					m_swi.push_back(d);
				}
			}
			sw->close();
		}

		return true;
	}

	bool canopy_data::read_shaders(IReader* level)
	{
		IReader* chunk = open_chunk_checked(level, fsL_SHADERS);
		if (!chunk || chunk->elapsed() < 4)
		{
			Msg("! [perception] canopy: level shader table missing - no canopy field");
			if (chunk)
				chunk->close();
			return false;
		}

		const u32 n = chunk->r_u32();
		if (u64(n) > u64(remaining_of(chunk)))
		{
			Msg("! [perception] canopy: shader count %u implausible for %u bytes", n,
				remaining_of(chunk));
			chunk->close();
			return false;
		}
		m_shader.resize(n);
		m_texture.resize(n);
		for (u32 i = 0; i < n && chunk->elapsed() > 0; ++i)
		{
			string512 entry;
			if (!read_stringz(chunk, entry, sizeof(entry)))
				break;
			if (!entry[0])
				continue;

			// "<shader>/<texture>[,<lmap>...]"
			LPSTR slash = strchr(entry, '/');
			if (!slash)
				continue;
			*slash = 0;
			LPSTR tex = slash + 1;
			LPSTR comma = strchr(tex, ',');
			if (comma)
				*comma = 0;

			m_shader[i] = entry;
			m_texture[i] = tex;
		}
		chunk->close();
		return true;
	}

	bool canopy_data::read_visuals(IReader* level)
	{
		IReader* visuals = open_chunk_checked(level, fsL_VISUALS);
		if (!visuals)
		{
			Msg("! [perception] canopy: level has no visuals chunk");
			return false;
		}

		// Walked by hand rather than with IReader::open_chunk_iterator, which sizes the child reader
		// from the file without checking it against the parent and decompresses compressed chunks
		// before any check here could run. Mirrors open_chunk_checked: bound the size, refuse
		// compression. The compress mark lives on the chunk id; the size is tested too in case a
		// corrupt file sets it there.
		while (remaining_of(visuals) >= 8)
		{
			const u32 cid = visuals->r_u32();
			const u32 size = visuals->r_u32();
			if ((cid & CFS_CompressMark) || (size & CFS_CompressMark) || size > remaining_of(visuals))
			{
				Msg("! [perception] canopy: visuals chunk is compressed or truncated - remaining visuals skipped");
				++m_skipped;
				break;
			}

			IReader* v = xr_new<IReader>(visuals->pointer(), int(size), visuals->tell() + int(size));
			read_visual(v);
			v->close();
			visuals->advance(int(size));
		}
		visuals->close();
		return true;
	}

	void canopy_data::read_visual(IReader* v)
	{
		// Only type and shader_id are wanted, read as bytes so no assumption is made about the
		// rest of the header's layout.
		const u8* h = chunk_at(v, OGF_HEADER, 4);
		if (!h)
			return;
		const u8 type = h[1];
		const u16 shader_id = *(const u16*)(h + 2);

		const bool tree = (MT_TREE_ST == type || MT_TREE_PM == type);
		LPCSTR shader = (shader_id < m_shader.size() && m_shader[shader_id].size())
							? m_shader[shader_id].c_str()
							: 0;
		if (!is_leaf_shader(shader))
			return;
		if (!tree)
		{
			++m_non_tree_leaf;
			return;
		}

		const u8* g = chunk_at(v, OGF_GCONTAINER, 24);
		if (!g)
		{
			++m_skipped;
			return;
		}
		const u32* gc = (const u32*)g;

		const u8* td = chunk_at(v, OGF_TREEDEF2, sizeof(Fmatrix));
		if (!td)
		{
			++m_skipped;
			return;
		}
		Fmatrix xform;
		CopyMemory(&xform, td, sizeof(Fmatrix));

		u32 swi = u32(-1);
		bool swi_unresolved = false;
		if (MT_TREE_PM == type)
		{
			const u8* sw = chunk_at(v, OGF_SWICONTAINER, 4);
			const u32 sw_id = sw ? *(const u32*)sw : u32(-1);
			if (sw_id < m_swi.size() && m_swi[sw_id].tris)
				swi = sw_id;
			else
				swi_unresolved = true;
		}

		model_key k;
		k.vb = gc[0];
		k.vbase = gc[1];
		k.ib = gc[3];
		k.ibase = gc[4];
		k.icount = gc[5];
		k.swi = swi;
		k.shader_id = shader_id;

		if (k.vb >= m_vb.size() || k.ib >= m_ib.size() || !m_vb[k.vb].usable
			|| DECLTYPE_UNUSED == m_vb[k.vb].tc_type)
		{
			++m_skipped;
			return;
		}

		if (swi_unresolved)
			++m_unresolved_swi;

		u32 model = u32(-1);
		for (u32 i = 0; i < m_keys.size(); ++i)
			if (m_keys[i] == k)
			{
				model = i;
				break;
			}
		if (u32(-1) == model)
		{
			model = u32(m_keys.size());
			m_keys.push_back(k);
			canopy_model cm;
			cm.texture = (shader_id < m_texture.size()) ? m_texture[shader_id] : shared_str();
			cm.coverage = u32(-1);
			m_models.push_back(cm);
		}

		canopy_instance ci;
		ci.xform = xform;
		ci.model = model;
		m_instances.push_back(ci);
	}

	void canopy_data::decode_models()
	{
		for (u32 m = 0; m < m_keys.size(); ++m)
		{
			const model_key& k = m_keys[m];
			const vb_desc& vb = m_vb[k.vb];
			const ib_desc& ib = m_ib[k.ib];

			u32 i0 = k.ibase;
			u32 tris = k.icount / 3;
			if (u32(-1) != k.swi)
			{
				i0 = k.ibase + m_swi[k.swi].offset;
				tris = m_swi[k.swi].tris;
			}

			if (u64(i0) + u64(tris) * 3 > u64(ib.count))
			{
				Msg("! [perception] canopy: model %u index range past the buffer - skipped", m);
				++m_skipped;
				continue;
			}

			canopy_model& cm = m_models[m];
			cm.verts.reserve(tris * 3);
			cm.uvs.reserve(tris * 3);
			for (u32 t = 0; t < tris * 3; ++t)
			{
				const u32 vi = u32(ib.data[i0 + t]) + k.vbase;
				if (vi >= vb.count)
				{
					cm.verts.clear();
					cm.uvs.clear();
					Msg("! [perception] canopy: model %u index %u past VB %u - skipped", m, vi, k.vb);
					++m_skipped;
					break;
				}
				const u8* v = vb.data + u64(vi) * vb.stride;
				cm.verts.push_back(*(const Fvector*)(v + vb.pos_offset));
				cm.uvs.push_back(read_uv(v + vb.tc_offset, vb.tc_type));
			}
			drop_twins(cm);
		}

		release_geom();
	}

	void canopy_data::drop_twins(canopy_model& cm)
	{
		struct twin_key
		{
			s64 q[9]; // the three positions, quantised and in sorted order
			u32 tri;
		};

		const u32 n = u32(cm.verts.size() / 3);
		xr_vector<twin_key> keys(n);
		for (u32 t = 0; t < n; ++t)
		{
			s64 v[3][3];
			for (u32 j = 0; j < 3; ++j)
				for (u32 a = 0; a < 3; ++a)
					v[j][a] = s64(floor(double((&cm.verts[t * 3 + j].x)[a]) / TWIN_QUANT + 0.5));
			u32 o[3] = {0, 1, 2};
			std::sort(o, o + 3, [&](u32 l, u32 r) {
				return std::lexicographical_compare(v[l], v[l] + 3, v[r], v[r] + 3);
			});
			for (u32 j = 0; j < 3; ++j)
				for (u32 a = 0; a < 3; ++a)
					keys[t].q[j * 3 + a] = v[o[j]][a];
			keys[t].tri = t;
		}
		std::sort(keys.begin(), keys.end(), [](const twin_key& l, const twin_key& r) {
			for (u32 k = 0; k < 9; ++k)
				if (l.q[k] != r.q[k])
					return l.q[k] < r.q[k];
			return l.tri < r.tri;
		});

		xr_vector<u8> drop(n, 0);
		u32 dropped = 0;
		for (u32 i = 1; i < n; ++i)
			if (std::equal(keys[i].q, keys[i].q + 9, keys[i - 1].q))
			{
				drop[keys[i].tri] = 1;
				++dropped;
			}
		if (!dropped)
			return;

		u32 w = 0;
		for (u32 t = 0; t < n; ++t)
		{
			if (drop[t])
				continue;
			for (u32 j = 0; j < 3; ++j)
			{
				cm.verts[w * 3 + j] = cm.verts[t * 3 + j];
				cm.uvs[w * 3 + j] = cm.uvs[t * 3 + j];
			}
			++w;
		}
		cm.verts.resize(w * 3);
		cm.uvs.resize(w * 3);
		m_twins += dropped;
	}

	static IC u32 pow2_at_least(float x)
	{
		u32 r = 1;
		while (r < (1u << 16) && float(r) < x)
			r <<= 1;
		return r;
	}

	void canopy_data::resolve_coverage(float finest_cell, float alpha_ref)
	{
		// Mean scale of each model's placements, so a texel's size is known in world metres.
		xr_vector<float> scale(m_models.size(), 0.f);
		{
			xr_vector<u32> n(m_models.size(), 0);
			for (u32 i = 0; i < m_instances.size(); ++i)
			{
				const Fmatrix& x = m_instances[i].xform;
				const float det = x._11 * (x._22 * x._33 - x._23 * x._32) - x._12 * (x._21 * x._33 - x._23 * x._31)
					+ x._13 * (x._21 * x._32 - x._22 * x._31);
				scale[m_instances[i].model] += powf(_abs(det), 1.f / 3.f);
				++n[m_instances[i].model];
			}
			for (u32 m = 0; m < m_models.size(); ++m)
				scale[m] = n[m] ? scale[m] / float(n[m]) : 0.f;
		}

		// Several models share a texture (145 over 35 on Escape), and reading a DDS is the expensive
		// part of this bake, so each is built once, sized for every model that uses it.
		xr_vector<float> sizes;
		for (u32 m = 0; m < m_models.size(); ++m)
		{
			canopy_model& cm = m_models[m];
			if (cm.verts.empty())
				continue;
			if (!cm.texture.size())
			{
				++m_no_texture;
				continue;
			}

			bool seen = false;
			for (u32 p = 0; p < m; ++p)
				if (m_models[p].texture == cm.texture && !m_models[p].verts.empty())
				{
					cm.coverage = m_models[p].coverage;
					seen = true;
					break;
				}
			if (seen)
				continue;

			// Metres of placed card per UV unit, over every triangle of every model on this texture.
			sizes.clear();
			for (u32 p = m; p < m_models.size(); ++p)
			{
				const canopy_model& pm = m_models[p];
				if (pm.texture != cm.texture || !scale[p])
					continue;
				for (u32 t = 0; t < pm.verts.size() / 3; ++t)
				{
					const Fvector* v = &pm.verts[t * 3];
					const Fvector2* uv = &pm.uvs[t * 3];
					Fvector e1, e2, c;
					e1.sub(v[1], v[0]);
					e2.sub(v[2], v[0]);
					const float area = c.crossproduct(e1, e2).magnitude();
					const float uv_area = _abs((uv[1].x - uv[0].x) * (uv[2].y - uv[0].y)
					                           - (uv[1].y - uv[0].y) * (uv[2].x - uv[0].x));
					if (area > 1e-8f && uv_area > 1e-12f)
						sizes.push_back(_sqrt(area / uv_area) * scale[p]);
				}
			}

			IReader* r = open_texture(cm.texture.c_str());
			dds_info info;
			const bool probed = r && dds_probe((const u8*)r->pointer(), u32(r->length()), info);

			u32 res = 1;
			if (probed)
			{
				float s90 = 0.f;
				if (!sizes.empty())
				{
					const size_t k = sizes.size() * 9 / 10;
					std::nth_element(sizes.begin(), sizes.begin() + k, sizes.end());
					s90 = sizes[k];
				}
				res = pow2_at_least(s90 / _max(finest_cell, 1e-3f));
				while (res > 1 && (res > info.width || res > info.height))
					res >>= 1;
			}

			xr_vector<u8> base;
			coverage_lod lod;
			if (probed && dds_coverage_grid((const u8*)r->pointer(), u32(r->length()), res, alpha_ref, base))
				lod.build(base, res);
			if (r)
				FS.r_close(r);

			if (!lod.valid())
			{
				++m_rejected;
				continue;
			}

			cm.coverage = u32(m_coverage.size());
			m_coverage.push_back(lod);
			PDBG_LOG("~ [perception] canopy:   tex '%s' %ux%u -> coverage %ux%u, mean %.3f", cm.texture.c_str(),
			         info.width, info.height, res, res, lod.mean());
		}
	}
}
