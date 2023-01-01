/**
 * WISE_Scenario_Growth_Module: ScenarioIgnition.cpp
 * Copyright (C) 2023  WISE
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "angles.h"
#include "ScenarioIgnition.h"
#include "FireEngine_ext.h"
#include "poly.h"


template<class _type>
IgnitionNode<_type>::IgnitionNode(WTimeManager *tm) :
    m_ignitionTime((uint64_t)0, tm),
		m_ignitionHasPoint(false),
		m_ignitionHasPoly(false) {
}


template<class _type>
IgnitionNode<_type>::IgnitionNode(const IgnitionNode<_type> &toCopy, WTimeManager *tm) : m_ignitionTime((uint64_t)0, tm) {
	m_ignitionCOM = toCopy.m_ignitionCOM;
	m_ignitionTime.SetTime(toCopy.m_ignitionTime);
	m_ignitionTime.SetTimeManager(tm);
	m_ignitionHasPoint = toCopy.m_ignitionHasPoint;
	m_ignitionHasPoly = toCopy.m_ignitionHasPoly;
}


template<class _type>
void IgnitionNode<_type>::Reset(const WTime& startTimeOverride, const WTimeSpan &dt) {
	if (startTimeOverride.GetTime(0))
		m_ignitionTime = startTimeOverride;
	else
		m_ignitionCOM->GetIgnitionTime(&m_ignitionTime);
	m_ignitionTime += dt;

	std::uint32_t i, cnt;
	m_ignitionHasPoint = false;
	m_ignitionHasPoly = false;
	std::uint16_t type;
	m_ignitionCOM->GetIgnitionCount(&cnt);
	for (i = 0; i < cnt; i++) {
		m_ignitionCOM->GetIgnitionType(i, &type);
		if (type == CWFGM_FIRE_IGNITION_POINT)
			m_ignitionHasPoint = true;
		else if ((type == CWFGM_FIRE_IGNITION_POLYGON_IN) || (type == CWFGM_FIRE_IGNITION_POLYGON_OUT))
			m_ignitionHasPoly = true;
		if (m_ignitionHasPoint && m_ignitionHasPoly)
			break;
	}
}


template<class _type>
bool IgnitionNode<_type>::getPoint(_type dx, _type dy, XY_PointTempl<_type>& loc) const {
	std::uint32_t max_size;

	if (FAILED(m_ignitionCOM->GetIgnitionSize(0, &max_size)))
		return false;
	std::uint16_t ignitionType;
	std::uint32_t ii, ii_cnt;
	XY_Poly ig(max_size);

	if (FAILED(m_ignitionCOM->GetIgnition(0, &ignitionType, &ig)))
		return false;
	if (ig.NumPoints() > 1)
		return false;

	if ((dx != 0.0) || (dy != 0.0)) {
		XY_PointTempl<_type> dd(dx, dy);
		ig.TranslateXY(dd);
	}

	if (ignitionType == CWFGM_FIRE_IGNITION_POINT) {
		loc.x = ig.GetPoint(0).x;
		loc.y = ig.GetPoint(0).y;
		return true;
	}
	return false;
}


template class IgnitionNode<fireengine_float_type>;
