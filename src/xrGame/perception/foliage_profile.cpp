#include "stdafx.h"
#include "foliage_profile.h"
#include "dds_alpha.h"
#include "perception_debug.h"

namespace perception
{
	// Same alias order CTexture::Load uses (Layers\xrRender\Texture.cpp). $level$ has to come first:
	// the detail atlas named in level.details ("build_details") ships inside the level folder, not in
	// the global texture archive, so a $game_textures$-only lookup silently misses every grass model.
	IReader* open_texture(LPCSTR name)
	{
		if (!name || !name[0])
			return 0;
		if (xr_strlen(name) >= sizeof(string_path))
			return 0;

		string_path fname;
		xr_strcpy(fname, name);
		if (LPSTR ext = strext(fname))
			if (0 == stricmp(ext, ".dds") || 0 == stricmp(ext, ".tga") || 0 == stricmp(ext, ".bmp"))
				*ext = 0;

		string_path fn;
		if (!FS.exist(fn, "$level$", fname, ".dds") &&
			!FS.exist(fn, "$game_saves$", fname, ".dds") &&
			!FS.exist(fn, "$game_textures$", fname, ".dds"))
			return 0;

		return FS.r_open(fn);
	}

	bool texture_mean_alpha(LPCSTR name, const uv_rect& uv, float& out_alpha, dds_info* out_info,
	                        float* out_gap)
	{
		IReader* r = open_texture(name);
		if (!r)
			return false;

		const u8* data = (const u8*)r->pointer();
		const u32 size = u32(r->length());

		if (out_info)
			dds_probe(data, size, *out_info);

		const bool ok = dds_mean_alpha(data, size, uv, out_alpha, 64, out_gap);
		FS.r_close(r);
		return ok;
	}

#ifdef PERCEPTION_DEBUG
	static void log_texture_info(LPCSTR name, IReader* r)
	{
		dds_info i;
		if (!dds_probe((const u8*)r->pointer(), u32(r->length()), i))
		{
			Msg("! [perception] foliage: '%s' is not a DDS this parser understands (%d bytes)", name, r->length());
			return;
		}
		char cc[5] = {0};
		if (i.fourcc)
			*(u32*)cc = i.fourcc;
		else
			xr_strcpy(cc, "raw");
		Msg("~ [perception] foliage: atlas '%s' %dx%d mips %d fmt %s bits %d amask 0x%08x",
		    name, i.width, i.height, i.mips, cc, i.bits, i.amask);
	}
#endif

	void build_foliage_profiles(const detail_data& dd, xr_vector<foliage_model_profile>& out)
	{
		PDBG_SCOPE("foliage: model profiles");

		const xr_vector<detail_model_desc>& models = dd.models();
		out.resize(models.size());

		// vanilla points all models at one atlas; cache so an 11 MB read happens once, not 32 times
		shared_str cached_name;
		IReader* cached = 0;

		for (u32 i = 0; i < models.size(); ++i)
		{
			const detail_model_desc& m = models[i];
			foliage_model_profile& p = out[i];

			p.height = m.bbox.max.y - m.bbox.min.y;
			if (!_valid(p.height) || p.height < 0.f)
				p.height = 0.f;

			// Footprint radius for the cylinder proxy. The larger of the two horizontal extents, so a
			// flat crossed-quad model is not modelled as a sliver a ray can slip past edge-on.
			const float ex = m.bbox.max.x - m.bbox.min.x;
			const float ez = m.bbox.max.z - m.bbox.min.z;
			p.radius = _max(ex, ez) * .5f;
			if (!_valid(p.radius) || p.radius < 0.f)
				p.radius = 0.f;

			p.area = m.area;
			p.opacity = 0.f;
			p.gap = 0.f;

			if (cached_name != m.texture)
			{
				if (cached)
				{
					FS.r_close(cached);
					cached = 0;
				}
				cached = open_texture(m.texture.c_str());
				cached_name = m.texture;
				if (!cached)
				{
					Msg("! [perception] foliage: detail texture '%s' not found", m.texture.c_str());
				}
				else
				{
					PDBG_ONLY(log_texture_info(m.texture.c_str(), cached));
				}
			}

			if (cached)
			{
				const u8* data = (const u8*)cached->pointer();
				const u32 size = u32(cached->length());

				float w_sum = 0.f, a_sum = 0.f, g_sum = 0.f;
				for (u32 f = 0; f < m.faces.size(); ++f)
				{
					float a = 0.f, g = 0.f;
					if (!dds_mean_alpha(data, size, m.faces[f].uv, a, 64, &g))
						continue;
					a_sum += a * m.faces[f].area;
					g_sum += g * m.faces[f].area;
					w_sum += m.faces[f].area;
				}

				if (w_sum > EPS_S)
				{
					p.opacity = a_sum / w_sum;
					p.gap = g_sum / w_sum;
				}
				else
					dds_mean_alpha(data, size, m.uv, p.opacity, 64, &p.gap);

				if (!_valid(p.opacity))
					p.opacity = 0.f;
				if (!_valid(p.gap))
					p.gap = 0.f;
			}

			PDBG_LOG("~ [perception]   model %-3d h=%5.2fm r=%5.2fm a=%8.3f tris %-4d alpha=%.3f  %s",
			         i, p.height, p.radius, p.area, u32(m.faces.size()), p.opacity, m.texture.c_str());
		}

		if (cached)
			FS.r_close(cached);
	}
}
