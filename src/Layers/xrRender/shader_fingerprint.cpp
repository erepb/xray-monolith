#include "stdafx.h"
#include "shader_fingerprint.h"

namespace shader_fingerprint
{
	struct entry
	{
		shared_str name;
		u32 source_crc;
		u32 defines_hash;
	};

	// A handful of shaders are ever queried, so a linear scan over a small vector beats a map and
	// keeps this free of allocation churn on the compile path.
	static xr_vector<entry>& table()
	{
		static xr_vector<entry> s_table;
		return s_table;
	}

	void record(LPCSTR name, u32 source_crc, LPCSTR defines)
	{
		if (!name)
			return;

		const u32 dh = defines ? crc32(defines, xr_strlen(defines)) : 0;

		xr_vector<entry>& t = table();
		for (u32 i = 0; i < t.size(); ++i)
		{
			if (0 == xr_strcmp(t[i].name.c_str(), name))
			{
				t[i].source_crc = source_crc;
				t[i].defines_hash = dh;
				return;
			}
		}

		entry e;
		e.name = name;
		e.source_crc = source_crc;
		e.defines_hash = dh;
		t.push_back(e);
	}

	bool get(LPCSTR name, u32& source_crc, u32& defines_hash)
	{
		if (!name)
			return false;

		const xr_vector<entry>& t = table();
		for (u32 i = 0; i < t.size(); ++i)
		{
			if (0 == xr_strcmp(t[i].name.c_str(), name))
			{
				source_crc = t[i].source_crc;
				defines_hash = t[i].defines_hash;
				return true;
			}
		}
		return false;
	}
}
