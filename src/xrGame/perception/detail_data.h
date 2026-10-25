#pragma once

namespace perception
{
	// One triangle of a detail model with its texture coordinates.
	struct uv_face
	{
		float area; // model space
		Fvector p[3]; // model space
		float u[3], v[3];
	};

	struct detail_model_desc
	{
		shared_str shader;
		shared_str texture;
		Fbox bbox;
		float min_scale, max_scale;
		u32 verts;
		u32 tris;
		xr_vector<uv_face> faces;
	};

	struct detail_slot_desc
	{
		u16 id[4];
		u16 y_base;       // 20 cm units, world base = y_base*0.2 - 200
		u8 y_height;      // 10 cm units
	};

	static const u16 DETAIL_ID_EMPTY = 0xFFFF;

	// A chunk header's declared size is unvalidated in release (find_chunk's size check is DEBUG-only),
	// so a child reader's Size can exceed the bytes actually behind it - which would silently inflate
	// every remaining() guard downstream. find_chunk leaves the parent positioned at the child's data
	// start, so the parent's remaining count is the real bound. Resolves the chunk exactly once and
	// builds the child itself: IReader::open_chunk would re-resolve the id and can land on a different
	// chunk with the same id. Compressed chunks are refused.
	IReader* open_chunk_checked(IReader* parent, u32 id);

	// IReader::r_stringZ(char*,u32) measures its length guard against the string at chunk offset 0,
	// not at Pos, so it is an unbounded write for any Pos != 0. Bound it ourselves.
	bool read_stringz(IReader* r, char* dest, u32 cap);

	// Read-only view over level.details. Deliberately parses the file itself instead of reaching
	// into CDetailManager: the renderer does not retain per-model texture names (CDetail::Load
	// passes them straight to shader.create), and this keeps the perception module free of any
	// render-side dependency.
	class detail_data
	{
	public:
		detail_data();
		~detail_data();

		bool load();
		void unload();

		u32 version() const { return m_version; }
		u32 size_x() const { return m_size_x; }
		u32 size_z() const { return m_size_z; }
		int offs_x() const { return m_offs_x; }
		int offs_z() const { return m_offs_z; }
		u32 slot_count() const { return m_size_x * m_size_z; }

		const xr_vector<detail_model_desc>& models() const { return m_models; }

		void slot(u32 index, detail_slot_desc& out) const;

	private:
		bool read_models(IReader* fs);

		IReader* m_fs;
		IReader* m_slots_chunk; // held open: open_chunk may decompress into its own buffer
		const u8* m_slots;
		u32 m_slot_bytes;
		u32 m_version;
		u32 m_object_count;
		u32 m_size_x, m_size_z;
		int m_offs_x, m_offs_z;
		xr_vector<detail_model_desc> m_models;
	};
}
