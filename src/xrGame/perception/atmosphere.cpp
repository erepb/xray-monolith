#include "stdafx.h"
#include "atmosphere.h"
#include "perception_debug.h"
#include "../../xrEngine/IGame_Persistent.h"   // pulls Environment.h

namespace perception
{
	atmosphere& atmosphere::instance()
	{
		static atmosphere s_atmo;
		return s_atmo;
	}

	void atmosphere::on_level_unload()
	{
		m_lut.set_analytic();
		m_ready = false;
		m_unmeasured = false;
	}

	void atmosphere::on_level_load()
	{
		m_lut.set_analytic();
		m_ready = true;

		// One fetch, then a private copy. Null is a supported answer, not an error: R1/R2/R3 do not
		// probe, and a failed R4 probe reports itself and hands back the analytic curve.
		const atmo_lut* src = ::Render ? ::Render->atmosphere_lut() : 0;
		if (src)
			m_lut = *src;
		m_unmeasured = src && m_lut.analytic;

		// Name the reason, not just the outcome. "analytic curve" alone reads identically whether the
		// renderer cannot probe, the probe was switched off, or it ran and produced nothing - and the
		// three call for completely different responses from whoever is reading the log.
		LPCSTR why = !src ? "analytic curve (this renderer does not probe)"
		                  : m_lut.analytic ? "analytic curve (probe produced none - see the line above)"
		                                   : "measured curve";
		Msg("~ [perception] atmosphere: %s, %ux%u", why, m_lut.nx, m_lut.ny);
		if (!m_lut.analytic)
			Msg("~ [perception]   crc 0x%08x defines 0x%08x, channel disagreement %.4f, degenerate %u",
			    m_lut.source_crc, m_lut.defines_hash, m_lut.max_channel_disagreement,
			    m_lut.degenerate_cells);
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

		return m_lut.sample(normalized_distance(length), height);
	}
}
