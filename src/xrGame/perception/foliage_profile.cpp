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

	float tuft_density::extinction(const Fvector& q, float k_up, float k_flat) const
	{
		const float rho = _sqrt(q.x * q.x + q.z * q.z);
		if (rho >= radius || q.y < y_min || q.y >= y_max)
			return 0.f;
		const u32 ir = _min(u32(rho / radius * float(TUFT_NR)), TUFT_NR - 1);
		const u32 iy = _min(u32((q.y - y_min) / (y_max - y_min) * float(TUFT_NY)), TUFT_NY - 1);
		return k_up * upright[iy][ir] + k_flat * flat[iy][ir];
	}

	static const u32 FACE_LATTICE_MAX = 32; // samples per triangle edge, about one per mip-0 texel below this

	struct density_bake_stats
	{
		float mean_alpha_area; // the same faces weighted by plain alpha, for comparison
		float upright_area;
		float area_y[TUFT_NY];
	};

	// Where `share` of the summed bin weights is reached, bins spanning [lo, lo + n * step).
	static float quantile(const float* w, u32 n, float lo, float step, float share)
	{
		float total = 0.f;
		for (u32 i = 0; i < n; ++i)
			total += w[i];
		if (total <= 0.f)
			return lo;

		const float goal = share * total;
		float acc = 0.f;
		for (u32 i = 0; i < n; ++i)
		{
			if (w[i] > 0.f && acc + w[i] >= goal)
				return lo + step * (float(i) + (goal - acc) / w[i]);
			acc += w[i];
		}
		return lo + step * float(n);
	}

	static void bake_density(const detail_model_desc& m, const dds_alpha_view& view, float alpha_ref,
	                         tuft_density& d, density_bake_stats& st)
	{
		ZeroMemory(&d, sizeof(d));
		ZeroMemory(&st, sizeof(st));

		for (u32 f = 0; f < m.faces.size(); ++f)
			for (u32 k = 0; k < 3; ++k)
			{
				const Fvector& p = m.faces[f].p[k];
				d.radius = _max(d.radius, _sqrt(p.x * p.x + p.z * p.z));
			}
		d.y_min = m.bbox.min.y;
		d.y_max = m.bbox.max.y;
		if (d.radius <= 0.f || !(d.y_max > d.y_min))
			return;

		const float span = d.y_max - d.y_min;
		const float cutoff = alpha_ref * 255.f;
		const float tex_w = float(view.width()), tex_h = float(view.height());
		float area_r[TUFT_NR] = {};

		for (u32 fi = 0; fi < m.faces.size(); ++fi)
		{
			const uv_face& f = m.faces[fi];

			Fvector e1, e2, n;
			e1.sub(f.p[1], f.p[0]);
			e2.sub(f.p[2], f.p[0]);
			n.crossproduct(e1, e2);
			const float nm = n.magnitude();
			if (nm < EPS_S || f.area <= 0.f)
				continue;
			const float ny = n.y / nm;
			const float w_up = _sqrt(_max(0.f, 1.f - ny * ny));
			const float w_flat = _abs(ny);

			float edge = 0.f;
			for (u32 k = 0; k < 3; ++k)
			{
				const u32 j = (k + 1) % 3;
				const float du = (f.u[j] - f.u[k]) * tex_w;
				const float dv = (f.v[j] - f.v[k]) * tex_h;
				edge = _max(edge, _sqrt(du * du + dv * dv));
			}
			const u32 lat = _min(_max(u32(iCeil(edge)), 1u), FACE_LATTICE_MAX);
			const float inv = 1.f / float(lat);
			const float sample_area = f.area / float(lat * lat);

			// Centroids of the lat^2 congruent sub-triangles: lat(lat+1)/2 upright, lat(lat-1)/2 inverted.
			for (u32 i = 0; i < lat; ++i)
				for (u32 j = 0; i + j < lat; ++j)
					for (u32 inverted = 0; inverted < 2; ++inverted)
					{
						if (inverted && i + j + 2 > lat)
							break;
						const float off = inverted ? 2.f / 3.f : 1.f / 3.f;
						const float a = (float(i) + off) * inv;
						const float b = (float(j) + off) * inv;

						const float u = f.u[0] + (f.u[1] - f.u[0]) * a + (f.u[2] - f.u[0]) * b;
						const float v = f.v[0] + (f.v[1] - f.v[0]) * a + (f.v[2] - f.v[0]) * b;
						const u32 alpha = view.alpha(u, v);
						st.mean_alpha_area += sample_area * float(alpha) / 255.f;
						if (float(alpha) < cutoff)
							continue;

						Fvector q;
						q.mad(f.p[0], e1, a);
						q.mad(e2, b);
						const float rho = _sqrt(q.x * q.x + q.z * q.z);
						const u32 ir = _min(u32(rho / d.radius * float(TUFT_NR)), TUFT_NR - 1);
						const u32 iy = _min(u32(_max(q.y - d.y_min, 0.f) / span * float(TUFT_NY)), TUFT_NY - 1);

						d.upright[iy][ir] += sample_area * w_up;
						d.flat[iy][ir] += sample_area * w_flat;
						d.opaque_area += sample_area;
						area_r[ir] += sample_area;
						st.area_y[iy] += sample_area;
						st.upright_area += sample_area * w_up;
					}
		}

		const float dr = d.radius / float(TUFT_NR);
		const float dy = span / float(TUFT_NY);
		for (u32 ir = 0; ir < TUFT_NR; ++ir)
		{
			const float r0 = dr * float(ir), r1 = r0 + dr;
			const float inv_vol = 1.f / (PI * (r1 * r1 - r0 * r0) * dy);
			for (u32 iy = 0; iy < TUFT_NY; ++iy)
			{
				d.upright[iy][ir] *= inv_vol;
				d.flat[iy][ir] *= inv_vol;
			}
		}

		d.r50 = quantile(area_r, TUFT_NR, 0.f, dr, .5f);
		d.r90 = quantile(area_r, TUFT_NR, 0.f, dr, .9f);
		d.top95 = quantile(st.area_y, TUFT_NY, d.y_min, dy, .95f);
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

	void build_foliage_profiles(const detail_data& dd, float alpha_ref, xr_vector<foliage_model_profile>& out)
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

			// The range DetailManager_Decompress.cpp draws instance scale from.
			p.max_placement_scale = _max(m.min_scale * 0.5f, m.max_scale * 0.9f);
			if (!_valid(p.max_placement_scale) || p.max_placement_scale < 0.f)
				p.max_placement_scale = 0.f;
			ZeroMemory(&p.density, sizeof(p.density));
			density_bake_stats st;
			ZeroMemory(&st, sizeof(st));

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
				dds_alpha_view view;
				if (view.open((const u8*)cached->pointer(), u32(cached->length())))
					bake_density(m, view, alpha_ref, p.density, st);
				else
					Msg("! [perception] foliage: atlas '%s' has no readable mip 0 - model %u blocks nothing",
					    m.texture.c_str(), i);
			}

			PDBG_LOG("~ [perception]   model %-3d h=%5.2fm tris %-4d  %s", i, p.height, u32(m.faces.size()),
			         m.texture.c_str());
#ifdef PERCEPTION_DEBUG
			{
				const tuft_density& d = p.density;
				const float sum = d.opaque_area > 0.f ? d.opaque_area : 1.f;
				Msg("~ [perception]     density R %.2f y %.2f..%.2f  opaque %.3f m2 (mean alpha %.3f)  r50 %.2f r90 %.2f  top95 %.2f  upright %.0f %%",
				    d.radius, d.y_min, d.y_max, d.opaque_area, st.mean_alpha_area, d.r50, d.r90, d.top95,
				    100.f * st.upright_area / sum);
				Msg("~ [perception]     y-profile  %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", st.area_y[0] / sum,
				    st.area_y[1] / sum, st.area_y[2] / sum, st.area_y[3] / sum, st.area_y[4] / sum,
				    st.area_y[5] / sum, st.area_y[6] / sum, st.area_y[7] / sum);
			}
#endif
		}

		if (cached)
			FS.r_close(cached);
	}
}
