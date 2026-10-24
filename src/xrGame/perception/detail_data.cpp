#include "stdafx.h"
#include "detail_data.h"
#include "perception_debug.h"
#include "../../Layers/xrRender/DetailFormat.h"

namespace perception
{
	// On-disk slot layouts. Mirrored here (rather than used directly) so the module needs no
	// render-side types at runtime; the asserts below catch format drift against the header values.
	static const u32 SLOT_BYTES_V3 = 16;
	static const u32 SLOT_BYTES_V4 = 20;
	static const u32 ID_EMPTY_V3 = 0x3F;
	static const u32 ID_EMPTY_V4 = 0x3FFF;

	static_assert(sizeof(DetailSlot_v3) == SLOT_BYTES_V3, "v3 slot layout drifted");
	static_assert(sizeof(DetailSlot) == SLOT_BYTES_V4, "v4 slot layout drifted");
	static_assert(DetailSlot_v3::ID_Empty == ID_EMPTY_V3, "v3 empty id drifted");
	static_assert(DetailSlot::ID_Empty == ID_EMPTY_V4, "v4 empty id drifted");

	struct detail_header_disk
	{
		u32 version;
		u32 object_count;
		int offs_x, offs_z;
		u32 size_x, size_z;
	};

	static_assert(sizeof(detail_header_disk) == sizeof(DetailHeader), "detail header layout drifted");

	static IC u32 rd32(const u8* p, u32 off) { return *(const u32*)(p + off); }
	static IC u32 remaining(IReader* r) { const int e = r->elapsed(); return e > 0 ? u32(e) : 0; }

	IReader* open_chunk_checked(IReader* parent, u32 id)
	{
		if (!parent)
			return 0;

		// One find_chunk, one child. IReader::open_chunk calls find_chunk a second time, and its
		// m_last_pos fast path can resolve a different chunk carrying the same id - one whose
		// declared size was never validated here. Compressed chunks are refused rather than
		// decompressed: _decompressLZ would run before any check of ours could.
		BOOL compressed = FALSE;
		const u32 sz = parent->find_chunk(id, &compressed);
		if (!sz || u64(sz) > u64(remaining(parent)))
			return 0;
		if (compressed)
		{
			Msg("! [perception] chunk %d is compressed - not supported, skipping", id);
			return 0;
		}
		return xr_new<IReader>(parent->pointer(), int(sz), parent->tell() + int(sz));
	}

	bool read_stringz(IReader* r, char* dest, u32 cap)
	{
		const u32 left = remaining(r);
		if (!left)
			return false;
		const char* src = (const char*)r->pointer();
		u32 n = 0;
		while (n < left && src[n])
			++n;
		if (n >= left || n >= cap) // no terminator before the end, or too long for dest
			return false;
		CopyMemory(dest, src, n);
		dest[n] = 0;
		r->advance(int(n) + 1);
		return true;
	}

	detail_data::detail_data()
		: m_fs(0), m_slots_chunk(0), m_slots(0), m_slot_bytes(0), m_version(0), m_object_count(0),
		  m_size_x(0), m_size_z(0), m_offs_x(0), m_offs_z(0)
	{
	}

	detail_data::~detail_data()
	{
		unload();
	}

	void detail_data::unload()
	{
		if (m_slots_chunk)
		{
			m_slots_chunk->close();
			m_slots_chunk = 0;
		}
		if (m_fs)
		{
			FS.r_close(m_fs);
			m_fs = 0;
		}
		m_slots = 0;
		m_object_count = 0;
		m_models.clear();
	}

	bool detail_data::load()
	{
		unload();

		if (!FS.exist("$level$", "level.details"))
		{
			PDBG_LOG("~ [perception] level.details absent - level has no detail objects");
			return false;
		}

		string_path fn;
		FS.update_path(fn, "$level$", "level.details");
		m_fs = FS.r_open(fn);
		if (!m_fs)
			return false;

		IReader* hdr = open_chunk_checked(m_fs, 0);
		if (!hdr || hdr->length() < int(sizeof(detail_header_disk)))
		{
			if (hdr) hdr->close();
			unload();
			return false;
		}
		detail_header_disk h;
		hdr->r(&h, sizeof(h));
		hdr->close();

		if (h.version != 3 && h.version != 4)
		{
			Msg("! [perception] unsupported level.details version %u", h.version);
			unload();
			return false;
		}

		if (h.object_count > 16383)
		{
			Msg("! [perception] level.details object_count implausible: %u", h.object_count);
			unload();
			return false;
		}

		static const u32 MAX_GRID = 8192; // 2 m slots; largest shipped level is ~2000 per axis
		if (!h.size_x || !h.size_z || h.size_x > MAX_GRID || h.size_z > MAX_GRID)
		{
			Msg("! [perception] level.details grid implausible: %ux%u", h.size_x, h.size_z);
			unload();
			return false;
		}

		m_version = h.version;
		m_object_count = h.object_count;
		m_size_x = h.size_x;
		m_size_z = h.size_z;
		m_offs_x = h.offs_x;
		m_offs_z = h.offs_z;
		m_slot_bytes = (h.version == 3) ? SLOT_BYTES_V3 : SLOT_BYTES_V4;

		if (!read_models(m_fs))
		{
			unload();
			return false;
		}

		m_slots_chunk = open_chunk_checked(m_fs, 2);
		if (!m_slots_chunk)
		{
			unload();
			return false;
		}
		const int chunk_len = m_slots_chunk->length();
		const u64 need = u64(slot_count()) * m_slot_bytes;
		if (chunk_len <= 0 || u64(chunk_len) < need)
		{
			Msg("! [perception] level.details slot chunk truncated: have %d need %llu", chunk_len, need);
			unload();
			return false;
		}
		m_slots = (const u8*)m_slots_chunk->pointer();

		return true;
	}

	static bool fail_model(IReader* m, IReader* container)
	{
		if (m) m->close();
		container->close();
		return false;
	}

	bool detail_data::read_models(IReader* fs)
	{
		IReader* container = open_chunk_checked(fs, 1);
		if (!container)
		{
			Msg("! [perception] level.details model container missing or invalid (object_count %u)", m_object_count);
			return false;
		}

		m_models.reserve(m_object_count);
		for (u32 m_id = 0; m_id < m_object_count; ++m_id)
		{
			IReader* m = open_chunk_checked(container, m_id);
			if (!m)
			{
				Msg("! [perception] level.details model chunk %u missing or invalid", m_id);
				return fail_model(m, container);
			}

			detail_model_desc d;
			string512 sh, tex;
			if (!read_stringz(m, sh, sizeof(sh)) || !read_stringz(m, tex, sizeof(tex)))
			{
				Msg("! [perception] level.details model %u truncated", m_id);
				return fail_model(m, container);
			}
			d.shader = sh;
			d.texture = tex;

			if (remaining(m) < 20)   // u32 flags + 2 floats + 2 u32
			{
				Msg("! [perception] level.details model %u truncated", m_id);
				return fail_model(m, container);
			}

			m->r_u32();   // flags
			m->r_float(); // min scale
			m->r_float(); // max scale

			d.verts = m->r_u32();
			const u32 n_idx = m->r_u32();
			d.tris = n_idx / 3;

			d.bbox.invalidate();
			d.uv.invalidate();
			d.area = 0.f;

			if (d.verts > remaining(m) / 20)
			{
				Msg("! [perception] level.details model %u vertex count implausible: %u", m_id, d.verts);
				return fail_model(m, container);
			}
			const u32 v_bytes = d.verts * 20;

			const u8* vraw = (const u8*)m->pointer();
			for (u32 v = 0; v < d.verts; ++v)
			{
				const u8* p = vraw + v * 20;
				Fvector pos;
				pos.set(*(const float*)(p + 0), *(const float*)(p + 4), *(const float*)(p + 8));
				d.bbox.modify(pos);
				d.uv.modify(*(const float*)(p + 12), *(const float*)(p + 16));
			}
			m->advance(v_bytes);

			if (n_idx <= remaining(m) / sizeof(u16))
			{
				const u16* idx = (const u16*)m->pointer();
				d.faces.reserve(d.tris);
				for (u32 t = 0; t + 2 < n_idx; t += 3)
				{
					if (idx[t] >= d.verts || idx[t + 1] >= d.verts || idx[t + 2] >= d.verts)
						continue;
					Fvector a, b, c, e1, e2, cr;
					const u8* p0 = vraw + idx[t] * 20;
					const u8* p1 = vraw + idx[t + 1] * 20;
					const u8* p2 = vraw + idx[t + 2] * 20;
					a.set(*(const float*)(p0 + 0), *(const float*)(p0 + 4), *(const float*)(p0 + 8));
					b.set(*(const float*)(p1 + 0), *(const float*)(p1 + 4), *(const float*)(p1 + 8));
					c.set(*(const float*)(p2 + 0), *(const float*)(p2 + 4), *(const float*)(p2 + 8));
					e1.sub(b, a);
					e2.sub(c, a);
					cr.crossproduct(e1, e2);

					uv_face f;
					f.area = 0.5f * cr.magnitude();
					f.uv.invalidate();
					f.uv.modify(*(const float*)(p0 + 12), *(const float*)(p0 + 16));
					f.uv.modify(*(const float*)(p1 + 12), *(const float*)(p1 + 16));
					f.uv.modify(*(const float*)(p2 + 12), *(const float*)(p2 + 16));

					if (!_valid(f.area))
						continue;
					d.faces.push_back(f);

					d.area += f.area;
				}
			}
			else
			{
				Msg("! [perception] level.details model %u index data truncated - per-face sampling skipped", m_id);
			}

			m_models.push_back(d);
			m->close();
		}

		container->close();
		return !m_models.empty();
	}

	void detail_data::slot(u32 index, detail_slot_desc& out) const
	{
		ZeroMemory(&out, sizeof(out));
		out.id[0] = out.id[1] = out.id[2] = out.id[3] = DETAIL_ID_EMPTY;

		if (!m_slots || index >= slot_count())
			return;

		const u8* p = m_slots + index * m_slot_bytes;

		// These bit offsets duplicate DetailFormat.h's DetailSlot_v3/DetailSlot bitfield layout and
		// must be updated together with it. The static_asserts above catch size and sentinel drift but
		// not a reorder at the same size - DetailFormat.h itself records that v4 already reordered its
		// fields once to pack into 3 x u32.
		if (3 == m_version)
		{
			const u32 w0 = rd32(p, 0);
			const u32 w1 = rd32(p, 4);
			out.y_base = u16(w0 & 0xFFF);
			out.y_height = u8((w0 >> 12) & 0xFF);
			const u32 raw[4] = {(w0 >> 20) & 0x3F, (w0 >> 26) & 0x3F, w1 & 0x3F, (w1 >> 6) & 0x3F};
			for (u32 i = 0; i < 4; ++i)
				out.id[i] = (raw[i] == ID_EMPTY_V3) ? DETAIL_ID_EMPTY : u16(raw[i]);
		}
		else
		{
			const u32 w0 = rd32(p, 0);
			const u32 w1 = rd32(p, 4);
			const u32 w2 = rd32(p, 8);
			const u32 raw[4] = {w0 & 0x3FFF, (w0 >> 14) & 0x3FFF, w1 & 0x3FFF, (w1 >> 14) & 0x3FFF};
			for (u32 i = 0; i < 4; ++i)
				out.id[i] = (raw[i] == ID_EMPTY_V4) ? DETAIL_ID_EMPTY : u16(raw[i]);
			out.y_base = u16(w2 & 0xFFF);
			out.y_height = u8((w2 >> 12) & 0xFF);
		}
	}
}
