#include "stdafx.h"
#include "slot_walk.h"

namespace perception
{
	// Absolute ceiling, independent of the Manhattan bound below. Only a non-finite input could reach
	// it, and finite endpoints are the caller's contract.
	static const u32 MAX_SLOT_STEPS = 4096;

	slot_walk::slot_walk(const Fvector& from, const Fvector& to)
	{
		m_sx = m_sz = m_sx_end = m_sz_end = 0;
		m_step_x = m_step_z = 1;
		m_t_max_x = m_t_max_z = m_t_delta_x = m_t_delta_z = flt_max;
		m_reported = 0;
		m_max_steps = 0;
		m_t_enter = 0.f;
		m_t_exit = 0.f;

		m_dir.sub(to, from);
		m_len = m_dir.magnitude();
		m_valid = m_len >= EPS_L;
		m_done = !m_valid;
		if (!m_valid)
			return;
		m_dir.div(m_len);
		const Fvector& dir = m_dir;

		// Slot sx spans world X in [sx*SLOT_SIZE, sx*SLOT_SIZE+SLOT_SIZE) per DetailManager_CACHE.cpp
		// and DetailManager_Decompress.cpp - no half-slot bias.
		m_sx = iFloor(from.x / SLOT_SIZE);
		m_sz = iFloor(from.z / SLOT_SIZE);
		m_sx_end = iFloor(to.x / SLOT_SIZE);
		m_sz_end = iFloor(to.z / SLOT_SIZE);

		m_step_x = (dir.x > 0.f) ? 1 : -1;
		m_step_z = (dir.z > 0.f) ? 1 : -1;

		// Distance along the ray to the next cell boundary on each axis, and the spacing thereafter.
		m_t_max_x = m_t_max_z = flt_max;
		m_t_delta_x = m_t_delta_z = flt_max;
		if (_abs(dir.x) > EPS_S)
		{
			const float bx = float((dir.x > 0.f) ? (m_sx + 1) : m_sx) * SLOT_SIZE;
			m_t_max_x = (bx - from.x) / dir.x;
			m_t_delta_x = _abs(SLOT_SIZE / dir.x);
		}
		if (_abs(dir.z) > EPS_S)
		{
			const float bz = float((dir.z > 0.f) ? (m_sz + 1) : m_sz) * SLOT_SIZE;
			m_t_max_z = (bz - from.z) / dir.z;
			m_t_delta_z = _abs(SLOT_SIZE / dir.z);
		}

		// Manhattan cell distance, not Euclidean length/SLOT_SIZE: a DDA spends a separate step on
		// each axis crossing, so a diagonal ray needs up to sqrt(2)x more steps than the straight-line
		// count gives it, and running out silently drops the tail of the path.
		const u32 span = u32(_abs(m_sx_end - m_sx) + _abs(m_sz_end - m_sz)) + 2;
		m_max_steps = _min(span, MAX_SLOT_STEPS);

	}

	bool slot_walk::next()
	{
		if (m_done || m_reported >= m_max_steps)
			return false;

		if (m_reported)
		{
			if (m_sx == m_sx_end && m_sz == m_sz_end)
			{
				m_done = true;
				return false;
			}

			if (m_t_max_x < m_t_max_z)
			{
				if (m_t_max_x > m_len)
				{
					m_done = true;
					return false;
				}
				m_t_enter = m_t_max_x;
				m_t_max_x += m_t_delta_x;
				m_sx += m_step_x;
			}
			else
			{
				if (m_t_max_z > m_len)
				{
					m_done = true;
					return false;
				}
				m_t_enter = m_t_max_z;
				m_t_max_z += m_t_delta_z;
				m_sz += m_step_z;
			}
		}

		++m_reported;
		m_t_exit = _min(_min(m_t_max_x, m_t_max_z), m_len);
		return true;
	}
}
