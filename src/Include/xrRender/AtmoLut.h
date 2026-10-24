#pragma once

// Measured atmospheric transmittance curve.
//
// Produced by the renderer (which is the only thing that can invoke the fog shader) and consumed by
// the AI, which must never touch the renderer at runtime. It is handed across that boundary exactly
// once per level load as plain data - the same shape as the Stage 1 foliage bake.
//
// The distance axis is NORMALIZED: the shader receives fog_params, already derived from
// fog_near/fog_far (Environment_misc.cpp:465-466), so every weather-dependent term enters there and
// the curve itself is a property of the shader alone. One measurement covers every weather preset.

struct atmo_lut
{
	enum { MAX_X = 64, MAX_Y = 32 };

	u16 nx, ny;
	float y_min, y_max;             // world-Y range spanned by the height axis
	float t[MAX_Y][MAX_X];          // transmittance 0..1, [height][distance]

	// Provenance: which shader produced this. Identity for the log and the debug report -
	// deliberately not a cache key, because it cannot see runtime cvars like ps_ssfx_fog.
	u32 source_crc;
	u32 defines_hash;
	u8 renderer;

	// Diagnostics.
	float max_channel_disagreement; // 0 for a pure lerp; larger means scattering or a non-linear term
	u16 degenerate_cells;
	bool analytic;                  // true = not measured, see ATMOSPHERE-DESIGN 6.3

	void set_analytic()
	{
		nx = 33;
		ny = 1;
		y_min = -1000.f;
		y_max = 1000.f;
		for (u16 i = 0; i < nx; ++i)
			t[0][i] = 1.f - float(i) / float(nx - 1); // stock D3D linear fog
		source_crc = 0;
		defines_hash = 0;
		renderer = 0;
		max_channel_disagreement = 0.f;
		degenerate_cells = 0;
		analytic = true;
	}

	// x: normalized distance 0..1. y: world height. Bilinear, clamped at both edges.
	float sample(float x, float y) const
	{
		if (!nx || !ny)
			return 1.f;

		clamp(x, 0.f, 1.f);
		const float fx = x * float(nx - 1);
		const u16 x0 = u16(fx);
		const u16 x1 = (x0 + 1 < nx) ? u16(x0 + 1) : x0;
		const float tx = fx - float(x0);

		u16 y0 = 0, y1 = 0;
		float ty = 0.f;
		if (ny > 1 && y_max > y_min)
		{
			float v = (y - y_min) / (y_max - y_min);
			clamp(v, 0.f, 1.f);
			const float fy = v * float(ny - 1);
			y0 = u16(fy);
			y1 = (y0 + 1 < ny) ? u16(y0 + 1) : y0;
			ty = fy - float(y0);
		}

		const float a = t[y0][x0] + (t[y0][x1] - t[y0][x0]) * tx;
		const float b = t[y1][x0] + (t[y1][x1] - t[y1][x0]) * tx;
		return a + (b - a) * ty;
	}
};
