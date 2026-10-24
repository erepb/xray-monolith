#include "stdafx.h"
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
	static const u32 DECLTYPE_FLOAT3 = 2;
	static const u32 DECLUSAGE_POSITION = 0;
	static const u16 DECL_STREAM_END = 0xFF;
	static const u32 DECL_ELEMENT_BYTES = 8;

	// A card alpha of 1.0 would give infinite optical depth; nothing in the atlas is truly opaque and
	// a single card must never end a sight line on its own.
	static const float ALPHA_CEILING = 0.995f;

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
		// Same rule foliage_oracle::scan_leafiness applies, and for the same reason: flora\trunk_wave
		// is bark - opaque, and present on the very same tree visuals as the canopy.
		return s && 0 == strncmp(s, "flora", 5) && !strstr(s, "trunk");
	}

	canopy_data::canopy_data()
		: m_geom(0), m_triangles(0), m_non_tree_leaf(0), m_rejected(0), m_unresolved_swi(0),
		  m_no_texture(0), m_skipped(0)
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
		m_triangles = 0;
		m_non_tree_leaf = 0;
		m_rejected = 0;
		m_unresolved_swi = 0;
		m_no_texture = 0;
		m_skipped = 0;
	}

	bool canopy_data::load()
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
		resolve_alpha();
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

		if (k.vb >= m_vb.size() || k.ib >= m_ib.size() || !m_vb[k.vb].usable)
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
			cm.ext = 0.f;
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
			for (u32 t = 0; t < tris * 3; ++t)
			{
				const u32 vi = u32(ib.data[i0 + t]) + k.vbase;
				if (vi >= vb.count)
				{
					cm.verts.clear();
					Msg("! [perception] canopy: model %u index %u past VB %u - skipped", m, vi, k.vb);
					++m_skipped;
					break;
				}
				cm.verts.push_back(*(const Fvector*)(vb.data + u64(vi) * vb.stride + vb.pos_offset));
			}
			m_triangles += u32(cm.verts.size() / 3);
		}

		release_geom();
	}

	void canopy_data::resolve_alpha()
	{
		// Several models share a texture (200 models over ~40 textures on Escape), and reading a DDS
		// is the expensive part of this bake, so each is read once.
		xr_vector<shared_str> seen;
		xr_vector<float> seen_ext;

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

			bool hit = false;
			for (u32 i = 0; i < seen.size(); ++i)
				if (seen[i] == cm.texture)
				{
					cm.ext = seen_ext[i];
					hit = true;
					break;
				}
			if (hit)
				continue;

			uv_rect whole;
			whole.invalidate();
			float a = 0.f;
			// No minimum resolution, unlike the leafiness average in foliage_oracle: a retired frond
			// card is a 2x2 fully transparent stub, and alpha 0 is that crown's correct extinction.
			if (!texture_mean_alpha(cm.texture.c_str(), whole, a, 0, 0))
			{
				++m_rejected;
				cm.ext = 0.f;
			}
			else
			{
				// A card transmits (1-alpha), so its optical depth is -ln(1-alpha). Using alpha itself
				// would under-read the dense conifers: measured spruce and pine alpha is 0.406, where
				// -ln(0.594) = 0.521.
				clamp(a, 0.f, ALPHA_CEILING);
				cm.ext = -logf(1.f - a);
			}

			seen.push_back(cm.texture);
			seen_ext.push_back(cm.ext);

			PDBG_LOG("~ [perception] canopy:   tex '%s' alpha=%.3f ext=%.3f", cm.texture.c_str(), a,
			         cm.ext);
		}
	}
}
