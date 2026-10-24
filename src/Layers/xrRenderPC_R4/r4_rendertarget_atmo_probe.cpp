#include "stdafx.h"
#include "../xrRender/shader_fingerprint.h"

// Atmospheric probe - measures the fog curve the live combine shader actually produces.
//
// The engine never encodes a fog formula. Reading shader source is not a sound basis for one: the
// first attempt at this analysed mip_fog.h and produced a confident wrong answer, because that file
// is included by nothing while the live fog is inline in combine_1.ps behind an #ifdef with two
// different implementations. So this invokes the shader that will actually run, and measures it.
//
// `fog` is a local inside the pixel shader and cannot be read out. But fog_params is engine-owned,
// so the shader's own fog factor can be driven to its endpoints and the middle solved for:
//
//   pass A  fog forced to 0  ->  outA = C0                 (pre-fog colour)
//   pass B  the probe value  ->  outB = C0*(1-f) + F*f
//   pass C  fog forced to 1  ->  outC = F                  (fog colour)
//
//   f = (outB - outA) / (outC - outA)
//
// C0 and F both cancel, so every other shader input only has to be FROZEN across the three passes,
// not plausible. The assumption is not "fog is linear" or "fog is exponential" - only that fog is a
// blend controlled by fog_params, which is the engine's constant-buffer contract rather than any
// shader's design.
//
// What the ratio CANNOT see: a factor the shader multiplies in outside fog_params. GAMMA's live
// combine_1.ps builds fog = fogD(fog_params) * saturate(exp(-worldY*0.002)); pass C forces fogD to 1
// rather than fog, so the height factor appears in both numerator and denominator and divides out.
// The solve returns fogD exactly - correct for distance, blind to height. That is not a defect that
// can be engineered away here: fog_params is derived from fog_near/fog_far, which are weather
// properties with no height input, so no height term can ever reach the shader through this lever.
// It costs little in practice - the factor is 1 at ground level, where observers are - and the
// height axis is collapsed to one row rather than filled with nine identical ones.

namespace
{
	const u16 PROBE_NX = 33;          // normalized distance samples
	const u16 PROBE_NY = 1;           // height rows; see the header for why more would measure nothing
	const float PROBE_Y_MIN = -1000.f; // with ny == 1 atmo_lut::sample ignores the height axis
	const float PROBE_Y_MAX = 1000.f;
	// View depth written into the synthetic G-buffer. Any non-degenerate value does: the fog
	// coordinate is driven through fog_params, so nothing in the measurement depends on this.
	const float PROBE_DEPTH = 500.f;

	// Exposure fed to the combine VS in place of the measured one. The fog lerp is followed by a
	// tonemap, and a tonemap is not affine - so the ratio of output differences is only the ratio of
	// input differences to the extent the map is locally straight. Deliberately small: every smooth
	// map is affine to first order near zero, which keeps the solve valid for ANY tonemap rather than
	// only the one that happens to ship. At this amplitude the stock curve bends by under 0.1 %.
	const float PROBE_EXPOSURE = 0.02f;

	// Pre-fill for the probe target. Negative, so it cannot be confused with any shader output.
	const float PROBE_SENTINEL = -1.f;

	// Below this, outC and outA are too close to divide: either the fog line did not execute, or the
	// fabricated inputs were degenerate. Small because the probe now runs at low amplitude on a
	// 32-bit float target - it is a floor against dividing noise, not a signal-strength test.
	const float COND_EPS = 1e-4f;

	// A texture slot borrowed for the duration of the probe. Restoring is not optional: leaving a
	// G-buffer slot pointing at a dead texture would take the next real frame down with it.
	struct slot_hijack
	{
		ref_texture slot;
		ID3DBaseTexture* saved; // surface_get() add-refs, so we owe a release

		slot_hijack() : saved(0) {}

		bool take(LPCSTR name, ID3DBaseTexture* with)
		{
			slot.create(name);
			if (!slot)
				return false;
			saved = slot->surface_get();
			slot->surface_set(with);
			return true;
		}

		void give_back()
		{
			if (slot)
			{
				// Unconditionally, NULL included. "$user$tonemap" is genuinely null between frames -
				// phase_combine binds it on entry and clears it on exit (phase_combine.cpp:61, 748) -
				// so treating null as "nothing to restore" would leave the slot holding the probe's
				// fake exposure instead of putting it back the way it was found.
				slot->surface_set(saved);
				slot.destroy();
			}
			_RELEASE(saved);
		}
	};

	struct probe_res
	{
		u16 nx, ny;
		ref_rt rt_out;
		ID3DTexture2D* tex_pos;
		ID3DTexture2D* tex_lum;
		ID3DTexture2D* tex_albedo;
		ID3DTexture2D* tex_accum;
		ID3DTexture2D* tex_stage;
		ID3DBlendState* bs_opaque;
		ID3DDepthStencilState* dss_off;
		slot_hijack pos;    // "$user$position" - the synthetic G-buffer
		slot_hijack lum;    // "$user$tonemap"  - the exposure the combine VS reads
		slot_hijack albedo; // "$user$albedo"   - surface colour
		slot_hijack accum;  // "$user$accum"    - light accumulation

		probe_res()
			: nx(0), ny(0), tex_pos(0), tex_lum(0), tex_albedo(0), tex_accum(0), tex_stage(0),
			  bs_opaque(0), dss_off(0) {}
	};

	void probe_destroy(probe_res& r)
	{
		r.pos.give_back();
		r.lum.give_back();
		r.albedo.give_back();
		r.accum.give_back();
		_RELEASE(r.tex_pos);
		_RELEASE(r.tex_lum);
		_RELEASE(r.tex_albedo);
		_RELEASE(r.tex_accum);
		_RELEASE(r.tex_stage);
		_RELEASE(r.bs_opaque);
		_RELEASE(r.dss_off);
		r.rt_out.destroy();
	}

	// One nx*ny RGBA32F texture filled with a single value. The G-buffer slots the probe fabricates
	// are all this shape, and sizing them to the target keeps the MSAA `Load(pos2d)` path in range as
	// well as the `Sample(tc)` one.
	bool probe_fill(ID3DTexture2D** out, u16 nx, u16 ny, const Fvector4& v)
	{
		xr_vector<Fvector4> data(u32(nx) * u32(ny), v);

		D3D_TEXTURE2D_DESC d;
		ZeroMemory(&d, sizeof(d));
		d.Width = nx;
		d.Height = ny;
		d.MipLevels = 1;
		d.ArraySize = 1;
		d.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
		d.SampleDesc.Count = 1;
		d.Usage = D3D_USAGE_IMMUTABLE;
		d.BindFlags = D3D_BIND_SHADER_RESOURCE;

		D3D_SUBRESOURCE_DATA sub;
		sub.pSysMem = &data[0];
		sub.SysMemPitch = nx * sizeof(Fvector4);
		sub.SysMemSlicePitch = 0;

		return SUCCEEDED(HW.pDevice->CreateTexture2D(&d, &sub, out));
	}

	bool probe_create(probe_res& r, u16 nx, u16 ny, float depth)
	{
		r.nx = nx;
		r.ny = ny;

		r.rt_out.create("$user$atmo_probe", nx, ny, D3DFMT_A32B32G32R32F, 1);
		if (!r.rt_out || !r.rt_out->pRT)
			return false;

		{
			// The synthetic G-buffer. gbuffer_stage.h:144 packs the normal into .xy, view depth into
			// .z and hemi/mtl into .w - not a position, despite the slot's name. Only .z has to be
			// sane; the fog coordinate no longer comes from here.
			Fvector4 p;
			p.set(0.f, 0.f, depth, 1.f);
			if (!probe_fill(&r.tex_pos, nx, ny, p))
				return false;
		}

		{
			// Exposure. combine_1.vs does `scale = s_tonemap.Load(0).x` and hands it to the pixel
			// shader, which multiplies the finished colour by it - so a zero here blacks out all three
			// passes identically and the solve sees a shader that ignores fog_params. Zero is exactly
			// what it holds here: phase_combine binds the slot on entry and clears it on exit
			// (r4_rendertarget_phase_combine.cpp:61, 748), and the probe runs between frames.
			Fvector4 lum;
			lum.set(PROBE_EXPOSURE, PROBE_EXPOSURE, PROBE_EXPOSURE, PROBE_EXPOSURE);
			if (!probe_fill(&r.tex_lum, 1, 1, lum))
				return false;
		}

		{
			// Surface colour and light. Both slots are cold when a level loads - the deferred passes
			// that fill them have not run - and an unbound SRV reads as zero, which makes the pre-fog
			// colour black and every pass identical no matter what fog does. ATMOSPHERE-DESIGN 5.2
			// always called for these; the first implementation left them to whatever the last frame
			// happened to leave behind, which at load time is nothing.
			Fvector4 albedo, accum;
			albedo.set(.50f, .50f, .50f, .20f); // rgb albedo, w gloss
			accum.set(.60f, .60f, .60f, .30f);  // rgb diffuse light, w specular
			if (!probe_fill(&r.tex_albedo, nx, ny, albedo) || !probe_fill(&r.tex_accum, nx, ny, accum))
				return false;
		}

		{
			// The combine pass blends: bABlend TRUE, INVSRCALPHA/SRCALPHA (blender_combine.cpp:19-20),
			// so the framebuffer gets src*(1-a) + dst*a with a = skyblend = a function of fog. On a
			// real frame that is what lets the sky show through total fog; here it makes the output a
			// second, unknown function of the very quantity being solved for - and at a == 1 it throws
			// the shader's colour away entirely. The measurement needs the shader's raw output, so the
			// probe overrides the pass's blend state with a plain opaque one.
			// Every enum has to hold a legal value even with BlendEnable off - CreateBlendState
			// validates the whole descriptor, and a zeroed one is not valid. Same defaults as
			// dx10StateUtils::ResetDescription.
			D3D_BLEND_DESC bd;
			ZeroMemory(&bd, sizeof(bd));
			bd.AlphaToCoverageEnable = FALSE;
			bd.IndependentBlendEnable = FALSE;
			for (int i = 0; i < 8; ++i)
			{
				bd.RenderTarget[i].BlendEnable = FALSE;
				bd.RenderTarget[i].SrcBlend = D3D_BLEND_ONE;
				bd.RenderTarget[i].DestBlend = D3D_BLEND_ZERO;
				bd.RenderTarget[i].BlendOp = D3D_BLEND_OP_ADD;
				bd.RenderTarget[i].SrcBlendAlpha = D3D_BLEND_ONE;
				bd.RenderTarget[i].DestBlendAlpha = D3D_BLEND_ZERO;
				bd.RenderTarget[i].BlendOpAlpha = D3D_BLEND_OP_ADD;
				bd.RenderTarget[i].RenderTargetWriteMask = D3D_COLOR_WRITE_ENABLE_ALL;
			}

			if (FAILED(HW.pDevice->CreateBlendState(&bd, &r.bs_opaque)))
				return false;
		}

		{
			// The pass also arrives with the stencil test armed - "render where stencil >= 1", which is
			// how a real frame confines combine to pixels the G-buffer wrote (blender_combine.cpp:21).
			// The probe binds no depth-stencil view, which should disable the test on its own; pinning
			// it off removes the "should" from a pipeline this file otherwise specifies completely.
			D3D_DEPTH_STENCIL_DESC dd;
			ZeroMemory(&dd, sizeof(dd));
			dd.DepthEnable = FALSE;
			dd.DepthWriteMask = D3D_DEPTH_WRITE_MASK_ZERO;
			dd.DepthFunc = D3D_COMPARISON_ALWAYS;
			dd.StencilEnable = FALSE;
			dd.StencilReadMask = 0xff;
			dd.StencilWriteMask = 0xff;
			dd.FrontFace.StencilFailOp = D3D_STENCIL_OP_KEEP;
			dd.FrontFace.StencilDepthFailOp = D3D_STENCIL_OP_KEEP;
			dd.FrontFace.StencilPassOp = D3D_STENCIL_OP_KEEP;
			dd.FrontFace.StencilFunc = D3D_COMPARISON_ALWAYS;
			dd.BackFace = dd.FrontFace;

			if (FAILED(HW.pDevice->CreateDepthStencilState(&dd, &r.dss_off)))
				return false;
		}

		{
			D3D_TEXTURE2D_DESC d;
			ZeroMemory(&d, sizeof(d));
			d.Width = nx;
			d.Height = ny;
			d.MipLevels = 1;
			d.ArraySize = 1;
			d.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
			d.SampleDesc.Count = 1;
			d.Usage = D3D_USAGE_STAGING;
			d.BindFlags = 0;
			d.CPUAccessFlags = D3D_CPU_ACCESS_READ;

			if (FAILED(HW.pDevice->CreateTexture2D(&d, 0, &r.tex_stage)))
				return false;
		}

		// Same mechanism phase_combine uses for the env cubemaps.
		return r.pos.take(r2_RT_P, r.tex_pos)
			&& r.lum.take(r2_RT_luminance_cur, r.tex_lum)
			&& r.albedo.take(r2_RT_albedo, r.tex_albedo)
			&& r.accum.take(r2_RT_accum, r.tex_accum);
	}

	// Frozen, deliberately non-degenerate constants. Anything set here cancels out of the solve, so
	// it only has to be the SAME across all three passes - but it must not be zero, or the pre-fog
	// colour and the fog colour both collapse to black and the division becomes 0/0.
	void probe_set_frozen_constants()
	{
		Fmatrix eye;
		eye.identity();
		// Both spellings of view->world: vanilla combine uses neither, GAMMA's uses m_inv_V. Frozen
		// rather than correct - whatever they feed cancels, but leftovers from the last real frame
		// would not be reproducible.
		RCache.set_c("m_v2w", eye);
		RCache.set_c("m_inv_V", eye);
		RCache.set_c("L_ambient", 0.35f, 0.35f, 0.35f, 1.f);
		RCache.set_c("Ldynamic_color", 0.80f, 0.80f, 0.80f, 1.f);
		RCache.set_c("Ldynamic_dir", 0.f, -1.f, 0.f, 0.f);
		RCache.set_c("env_color", 0.50f, 0.50f, 0.50f, 1.f);
		RCache.set_c("fog_color", 0.05f, 0.05f, 0.05f, 1.f);
		RCache.set_c("ssao_noise_tile_factor", 0.f);
		RCache.set_c("ssao_kernel_size", 1.f);
	}
}

bool CRenderTarget::probe_atmosphere(atmo_lut& out)
{
	out.set_analytic();

	// CurrentEnv is created in CEnvironment::load (Environment_misc.cpp:858) - at level load, while
	// this class is built back in CRender::Create. The constant binders dereference it
	// unconditionally, so probing before a level exists would fault.
	if (!g_pGamePersistent || !g_pGamePersistent->Environment().CurrentEnv)
		return false;
	if (!s_combine || !s_combine->E[0] || !g_combine)
		return false;

	const u16 nx = PROBE_NX, ny = PROBE_NY;
	const u32 cells = u32(nx) * u32(ny);


	// The probe does not fabricate a distance. It fabricates the shader's own fog COORDINATE.
	//
	// Every fog line in this family reads `saturate(distance*fog_params.w + fog_params.x)`, and
	// fog_params is derived from fog_near/fog_far precisely so that expression is the normalized
	// position in the fog envelope - which is exactly the LUT's x axis. With w = 0 the distance term
	// drops out and the argument is x alone, so the probe hands the shader the coordinate it wants to
	// tabulate and nothing else:
	//
	//   pass A   x = 0        fog = 0
	//   pass B   x = u_i      fog = u_i          u_i = i / (nx-1)
	//   pass C   x = 1        fog = 1
	//
	// The first attempt instead wrote a distance into rt_Position and let the shader derive the
	// coordinate. That made the measurement depend on the whole reconstruction path -
	// `gbd.P = P.z*(pos2d*pos_decompression_params.zw - pos_decompression_params.xy)` in
	// gbuffer_stage.h:144 - and it came back a straight line of slope 1.75 rather than 1.0, because
	// `length(P)` picked up the view-ray fan of the corner pixels the probe rasterises into. Zeroing
	// pos_decompression_params was supposed to collapse that and silently did not: set_c by name is a
	// no-op when the constant misses the table (R_Backend.h:385). Driving fog_params.x directly
	// removes the entire dependency - position, FOV, aspect, screen location - rather than correcting
	// for it.
	//
	// u_i matches atmo_lut::sample's own indexing, so cell i is read back at exactly the coordinate it
	// was measured at.
	probe_res r;
	if (!probe_create(r, nx, ny, PROBE_DEPTH))
	{
		Msg("! [perception] atmo probe: resource creation failed");
		probe_destroy(r);
		return false;
	}

	// .y/.z are the near/far the engine also packs in here; no fog line in this family reads them.
	Fvector4 fp;
	fp.set(0.f, 0.f, PROBE_DEPTH, 0.f);

	xr_vector<Fvector4> smp[3];
	bool ok = true;

	// Coverage is decided by rasterizer state the probe does not own, and at level load that state
	// belongs to the loading screen. The scissor rectangle is the dangerous one: the UI leaves a rect
	// behind together with an override (dxUIRender.cpp:150) that set_Element cannot clear, because
	// SetRasterizerState re-applies it on every state change (dx10StateManager.cpp:72-73). Dropping
	// the override has to come first, or set_Scissor's disable is undone again a moment later. The UI
	// re-establishes its own on the next SetScissor call, so nothing needs restoring.
	StateManager.OverrideScissoring(false);

	// The output-merger bindings and the viewport are borrowed exactly like the texture slots are, and
	// have to be handed back for the same reason. u_setrt below leaves RT0 pointing at the probe
	// target and nulls RT1/RT2 and the depth buffer; releasing that target does not unbind it, so the
	// loading screen carries on drawing into a 33x1 offscreen surface and the frame goes black until
	// something else happens to rebind the backbuffer. pRT/pZB are private with only a getter for RT,
	// so the device is asked instead - which add-refs, hence the releases below. Captured here rather
	// than at the top because nothing before this point disturbs them, and the early returns above
	// would otherwise have to release.
	ID3DRenderTargetView* saved_rt[3] = { 0, 0, 0 };
	ID3DDepthStencilView* saved_zb = 0;
	const u32 saved_w = dwWidth, saved_h = dwHeight;
	D3D_VIEWPORT saved_vp;
	UINT saved_vp_n = 0;

	HW.pContext->OMGetRenderTargets(3, saved_rt, &saved_zb);
	HW.pContext->RSGetViewports(&saved_vp_n, 0);
	if (saved_vp_n)
	{
		saved_vp_n = 1;
		HW.pContext->RSGetViewports(&saved_vp_n, &saved_vp);
	}

	for (int pass = 0; pass < 3 && ok; ++pass)
	{
		u_setrt(nx, ny, r.rt_out->pRT, 0, 0, 0);

		// Sentinel, so "the shader wrote black" and "nothing was drawn at all" are different readings
		// rather than the same zero. Anything that survives to the readback was never covered.
		const float sentinel[4] = { PROBE_SENTINEL, PROBE_SENTINEL, PROBE_SENTINEL, PROBE_SENTINEL };
		HW.pContext->ClearRenderTargetView(r.rt_out->pRT, sentinel);

		// One draw per column, because fog_params is a constant and each column needs its own value.
		// Cheap enough not to matter: nx single-pixel draws against one readback per pass.
		for (u16 i = 0; i < nx && ok; ++i)
		{
			// u_setrt records the size in dwWidth/dwHeight but never sends a viewport to D3D -
			// phase_bloom sets and restores one by hand around its small targets (phase_bloom.cpp:394,
			// 547). Here it also does the column selection.
			D3D_VIEWPORT vp = { float(i), 0.f, 1.f, float(ny), 0.f, 1.f };
			HW.pContext->RSSetViewports(1, &vp);

			u32 Offset = 0;
			FVF::TL* pv = (FVF::TL*)RCache.Vertex.Lock(4, g_combine->vb_stride, Offset);
			pv->set(-1.f, 1.f, 0.f, 1.f, 0, 0.f, 1.f);
			pv++;
			pv->set(-1.f, -1.f, 0.f, 0.f, 0, 0.f, 0.f);
			pv++;
			pv->set(1.f, 1.f, 1.f, 1.f, 0, 1.f, 1.f);
			pv++;
			pv->set(1.f, -1.f, 1.f, 0.f, 0, 1.f, 0.f);
			pv++;
			RCache.Vertex.Unlock(4, g_combine->vb_stride);

			RCache.set_Element(s_combine->E[0]);
			RCache.set_Geometry(g_combine);
			probe_set_frozen_constants();

			const float u = (nx > 1) ? float(i) / float(nx - 1) : 0.f;
			fp.x = (pass == 0) ? 0.f : (pass == 2) ? 1.f : u;

			// Last, so all of these win over what set_Element bound. StateManager is applied inside
			// Render, so the overrides placed here are the ones that reach the pipeline.
			RCache.set_c("fog_params", fp);
			StateManager.SetBlendState(r.bs_opaque);
			StateManager.SetDepthStencilState(r.dss_off);
			RCache.set_Scissor(0);

			RCache.Render(D3DPT_TRIANGLELIST, Offset, 0, 4, 0, 2);
		}

		HW.pContext->CopyResource(r.tex_stage, r.rt_out->pSurface);

		D3D_MAPPED_TEXTURE2D m;
		if (FAILED(HW.pContext->Map(r.tex_stage, 0, D3D_MAP_READ, 0, &m)))
		{
			ok = false;
			break;
		}
		smp[pass].resize(cells);
		for (u16 j = 0; j < ny; ++j)
		{
			const Fvector4* row = (const Fvector4*)((const u8*)m.pData + m.RowPitch * j);
			for (u16 i = 0; i < nx; ++i)
				smp[pass][u32(j) * nx + i] = row[i];
		}
		HW.pContext->Unmap(r.tex_stage, 0);
	}

	// Hand everything back before the probe's own resources go away.
	u_setrt(saved_w, saved_h, saved_rt[0], saved_rt[1], saved_rt[2], saved_zb);
	if (saved_vp_n)
		HW.pContext->RSSetViewports(1, &saved_vp);
	for (u32 i = 0; i < 3; ++i)
		_RELEASE(saved_rt[i]);
	_RELEASE(saved_zb);

	// StateManager keeps weak links to whatever state objects it was last handed, so it has to stop
	// pointing at the probe's before they are released. Null is the D3D default state and every real
	// pass sets its own on the way through set_Element, so nothing downstream depends on this.
	StateManager.SetBlendState(0);
	StateManager.SetDepthStencilState(0);
	probe_destroy(r);

	if (!ok)
	{
		Msg("! [perception] atmo probe: pass or readback failed");
		return false;
	}

	u16 degenerate = 0;
	float worst_disagreement = 0.f;
	float widest_endpoint = 0.f; // largest |outC - outA| anywhere; only read on the failure path

	for (u16 j = 0; j < ny; ++j)
	{
		for (u16 i = 0; i < nx; ++i)
		{
			const u32 k = u32(j) * nx + i;

			const float da[3] = {
				smp[2][k].x - smp[0][k].x, smp[2][k].y - smp[0][k].y, smp[2][k].z - smp[0][k].z
			};
			const float nb[3] = {
				smp[1][k].x - smp[0][k].x, smp[1][k].y - smp[0][k].y, smp[1][k].z - smp[0][k].z
			};

			// Use the best-conditioned channel; a pure lerp gives all three the same answer, so any
			// spread means scattering or a non-linear term rather than a simple blend.
			int best = -1;
			float best_mag = COND_EPS;
			for (int ch = 0; ch < 3; ++ch)
			{
				widest_endpoint = _max(widest_endpoint, _abs(da[ch]));
				if (_abs(da[ch]) > best_mag)
				{
					best_mag = _abs(da[ch]);
					best = ch;
				}
			}

			if (best < 0)
			{
				// Carry the previous cell forward: monotone and harmless, and the count is reported so
				// nobody mistakes it for a measurement.
				++degenerate;
				out.t[j][i] = i ? out.t[j][i - 1] : 1.f;
				continue;
			}

			float f = nb[best] / da[best];
			clamp(f, 0.f, 1.f);

			for (int ch = 0; ch < 3; ++ch)
			{
				if (ch == best || _abs(da[ch]) <= COND_EPS)
					continue;
				float fc = nb[ch] / da[ch];
				clamp(fc, 0.f, 1.f);
				worst_disagreement = _max(worst_disagreement, _abs(fc - f));
			}

			out.t[j][i] = 1.f - f;
		}
	}

	if (degenerate > cells / 2)
	{
		// All cells degenerate means forcing fog_params changed nothing: either the combine shader in
		// use does not route its fog through fog_params at all, or the synthetic G-buffer did not
		// reach it. Either way there is nothing to measure and the analytic curve is the honest answer.
		Msg("! [perception] atmo probe: %u/%u cells degenerate - fog_params drove no visible change; "
		    "falling back to analytic", degenerate, cells);

		// Raw samples, because two rounds of this were spent guessing which of several plausible
		// inputs was flat. All zero: the shader ran but produced black, so something upstream of the
		// fog killed the colour. Equal and non-zero: it produced colour that fog_params does not
		// touch. Different across passes but under the floor: the endpoints are simply too close.
		// Alpha is printed because it carries skyblend, which the shader derives from fog independently
		// of the colour: alpha that moves while rgb does not says the fog ran and the colour path is
		// what is flat. All four channels equal to the sentinel says the draw never covered the target.
		const u32 mid = cells / 2;
		Msg("~ [perception]   widest |outC-outA| %.6f over %u cells (floor %.6f, sentinel %.1f)",
		    widest_endpoint, cells, COND_EPS, PROBE_SENTINEL);
		for (u32 pass = 0; pass < 3; ++pass)
			Msg("~ [perception]   pass %c  cell0 (%.5f %.5f %.5f a%.5f)  cell%u (%.5f %.5f %.5f a%.5f)",
			    "ABC"[pass],
			    smp[pass][0].x, smp[pass][0].y, smp[pass][0].z, smp[pass][0].w,
			    mid, smp[pass][mid].x, smp[pass][mid].y, smp[pass][mid].z, smp[pass][mid].w);
		out.set_analytic();
		return false;
	}

	out.nx = nx;
	out.ny = ny;
	out.y_min = PROBE_Y_MIN;
	out.y_max = PROBE_Y_MAX;
	out.max_channel_disagreement = worst_disagreement;
	out.degenerate_cells = degenerate;
	out.analytic = false;
	out.renderer = 4;
	out.source_crc = 0;
	out.defines_hash = 0;

	// Key on the pixel shader that actually ran, not on the blender's name: shader_compile records
	// under the compiled name, which carries the MSAA suffix ("combine_1_nomsaa").
	if (s_combine->E[0]->passes.size() && s_combine->E[0]->passes[0]->ps)
		shader_fingerprint::get(s_combine->E[0]->passes[0]->ps->cName.c_str(),
		                        out.source_crc, out.defines_hash);

	Msg("~ [perception] atmo probe: %ux%u measured, crc 0x%08x defines 0x%08x, disagreement %.4f, degenerate %u/%u",
	    nx, ny, out.source_crc, out.defines_hash, worst_disagreement, degenerate, cells);
	Msg("~ [perception]   T(x): 0.25 -> %.3f   0.50 -> %.3f   0.75 -> %.3f",
	    out.sample(.25f, 0.f), out.sample(.50f, 0.f), out.sample(.75f, 0.f));

	return true;
}
