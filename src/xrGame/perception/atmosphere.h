#pragma once

#include "occlusion_types.h"
#include "../../Include/xrRender/AtmoLut.h"

namespace perception
{
	// Distance-driven visibility, so NPCs inherit the limit fog already imposes on the player.
	//
	// Holds a private copy of the curve the renderer measured. The AI path never touches IRender
	// after that one fetch - same discipline as the Stage 1 foliage bake.
	class atmosphere
	{
	public:
		static atmosphere& instance();

		void on_level_load();
		void on_level_unload();

		bool ready() const { return m_ready; }
		bool is_analytic() const { return m_lut.analytic; }

		// The renderer probes, but the probe produced no curve. A renderer that does not probe at all
		// is not this case.
		bool unmeasured() const { return m_unmeasured; }
		const atmo_lut& lut() const { return m_lut; }

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
		atmo_lut m_lut;
		bool m_ready = false;
		bool m_unmeasured = false;
	};
}
