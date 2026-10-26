#pragma once

#include "occlusion_types.h"
#include "../../Include/xrRender/AtmoLut.h"

namespace perception
{
	// Distance-driven visibility, so NPCs inherit the limit fog already imposes on the player.
	//
	// Holds a private copy of the curve the renderer measured. The AI path never touches IRender; the
	// copy is refreshed from the main thread when the renderer re-measures (ATMOSPHERE-DESIGN 6.2b).
	class atmosphere
	{
	public:
		static atmosphere& instance();

		void on_level_load();
		void on_level_unload();
		void on_frame(); // main thread

		bool ready() const { return m_ready; }
		bool is_analytic() const { return lut().analytic; }
		const atmo_lut& lut() const { return m_lut[m_active]; }

		// The renderer probes, but the probe produced no curve. A renderer that does not probe at all
		// is not this case.
		bool unmeasured() const { return m_unmeasured; }

		// Transmittance over a path of `length` metres whose relevant height is `height`, under the
		// current weather. Weather enters only here, and only through CEnvDescriptor - never the
		// renderer.
		float transmittance(float length, float height) const;

		// Convenience for a whole segment. See resolve_height for why the height is a min.
		float transmittance(const segment& seg) const
		{
			return transmittance(seg.length, resolve_height(seg.from.y, seg.to.y));
		}

		// Transmittance along a path is physically reciprocal, so the pair must agree in both
		// directions. The shader takes height from the shaded pixel, which is fine for a renderer
		// with one viewpoint and wrong for a two-observer system. Taking the lower of the two ends is
		// symmetric by construction, and conservative - fog pools low.
		static float resolve_height(float y_a, float y_b) { return _min(y_a, y_b); }

		// Normalized distance into the fog envelope, 0 at fog_near and 1 at fog_far. Exposed so the
		// debug report can show the input alongside the output.
		static float normalized_distance(float length);

	private:
		void adopt(const atmo_lut* src);

		// Two copies so AI vision on the parallel thread never reads one being overwritten.
		atmo_lut m_lut[2];
		volatile u32 m_active = 0;
		u32 m_generation = 0;
		bool m_ready = false;
		bool m_unmeasured = false;
	};
}
