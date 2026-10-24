#pragma once

#include "../../build_config_defines.h"

namespace perception
{
	// The 2 m XZ lattice both vegetation fields are addressed on. Grass inherits it from
	// DETAIL_SLOT_SIZE (DetailFormat.h:13); canopy adopts it so one traversal serves both.
	static const float SLOT_SIZE = 2.f;

	// Walks the lattice cells a segment crosses, in WORLD slot indices - each field adds its own
	// origin offset. Reports every cell once, with the exact sub-interval of the ray inside it.
	//
	// t_enter/t_exit are what make a long ray cheap: tested against the ray's height over the whole
	// segment, a 200 m shot that drops 30 m overlaps nearly every cell's vegetation band and rejects
	// nothing. Per cell the span is centimetres.
	class slot_walk
	{
	public:
		// from and to must be finite: iFloor on NaN/Inf is undefined and nothing here screens them.
		// The direction and length are derived here, so callers take them from dir() and len() rather
		// than computing their own. A segment shorter than EPS_L is not walkable: valid() is false and
		// next() reports nothing.
		slot_walk(const Fvector& from, const Fvector& to);

		bool valid() const { return m_valid; }
		float len() const { return m_len; }
		const Fvector& dir() const { return m_dir; }

		// Advances to the next crossed cell. False when the segment is exhausted or not valid().
		bool next();

		int sx() const { return m_sx; }
		int sz() const { return m_sz; }
		float t_enter() const { return m_t_enter; }
		float t_exit() const { return m_t_exit; }

	private:
		int m_sx, m_sz;
		int m_sx_end, m_sz_end;
		int m_step_x, m_step_z;
		float m_t_max_x, m_t_max_z;
		float m_t_delta_x, m_t_delta_z;
		Fvector m_dir;
		float m_len;
		float m_t_enter, m_t_exit;
		u32 m_reported;
		u32 m_max_steps;
		bool m_done;
		bool m_valid;
	};
}
