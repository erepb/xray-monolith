#include "stdafx.h"
#include "atmosphere.h"
#include "perception_debug.h"
#include "../../xrEngine/IGame_Persistent.h"   // pulls Environment.h
#include "../Level.h"

namespace perception
{
	atmosphere& atmosphere::instance()
	{
		static atmosphere s_atmo;
		return s_atmo;
	}

	void atmosphere::on_level_unload()
	{
		m_lut[0].set_analytic();
		m_lut[1].set_analytic();
		m_active = 0;
		m_ready = false;
		m_unmeasured = false;
	}

	void atmosphere::adopt(const atmo_lut* src)
	{
		const u32 next = 1 - m_active;
		if (src)
			m_lut[next] = *src;
		else
			m_lut[next].set_analytic();
		m_active = next;
		m_unmeasured = src && m_lut[next].analytic;
	}

	void atmosphere::on_level_load()
	{
		m_ready = true;

		// The height axis spans the level's static geometry (ATMOSPHERE-DESIGN 5.2.1).
		const Fbox& bb = Level().ObjectSpace.GetBoundingVolume();

		// Null is a supported answer, not an error: R1/R2/R3 do not probe, and a failed R4 probe
		// reports itself and hands back the analytic curve.
		adopt(::Render ? ::Render->atmosphere_lut(bb.min.y, bb.max.y) : 0);
		m_generation = ::Render ? ::Render->atmosphere_generation() : 0;
		const atmo_lut& m = lut();

		// Name the reason, not just the outcome. "analytic curve" alone reads identically whether the
		// renderer cannot probe, the probe was switched off, or it ran and produced nothing - and the
		// three call for completely different responses from whoever is reading the log.
		LPCSTR why = !::Render || !m_generation ? "analytic curve (this renderer does not probe)"
		             : m.analytic ? "analytic curve (probe produced none - see the line above)"
		                          : "measured curve";
		Msg("~ [perception] atmosphere: %s, %ux%u", why, m.nx, m.ny);
		if (!m.analytic)
			Msg("~ [perception]   crc 0x%08x defines 0x%08x, channel disagreement %.4f, degenerate %u",
			    m.source_crc, m.defines_hash, m.max_channel_disagreement, m.degenerate_cells);
	}

	void atmosphere::on_frame()
	{
		if (!m_ready || !::Render)
			return;
		const u32 gen = ::Render->atmosphere_generation();
		if (gen == m_generation)
			return;
		m_generation = gen;
		adopt(::Render->atmosphere_lut_current());
		Msg("~ [perception] atmosphere: re-measured curve adopted, %ux%u", lut().nx, lut().ny);
	}

	float atmosphere::normalized_distance(float length)
	{
		if (!g_pGamePersistent)
			return 0.f;

		// Mixer, not the base descriptor: fog_near/fog_far are derived members on CEnvDescriptorMixer
		// (Environment.h:230-242), while fog_density/fog_distance sit on the base.
		const CEnvDescriptorMixer* e = g_pGamePersistent->Environment().CurrentEnv;
		if (!e)
			return 0.f;

		// fog_near/fog_far are derived from the weather preset in Environment_misc.cpp:465-466. All
		// weather dependence enters here, which is what makes one measured curve cover every preset.
		const float n = e->fog_near;
		const float f = e->fog_far;
		if (f <= n + EPS_L)
			return 0.f;

		float x = (length - n) / (f - n);
		clamp(x, 0.f, 1.f);
		return x;
	}

	float atmosphere::transmittance(float length, float height) const
	{
		if (!m_ready || length <= 0.f)
			return 1.f;

		return lut().sample(normalized_distance(length), height);
	}
}
