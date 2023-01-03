/**
 * WISE_Scenario_Growth_Module: firestatecache.cpp
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
#include "macros.h"
#include "firestatecache.h"
#include "FireEngine_ext.h"
#include "FuelCom_ext.h"
#include "ICWFGM_FBPFuel.h"
#include "GridCom_ext.h"
#include "ScenarioTimeStep.h"
#include "CWFGM_Scenario_Internal.h"
#include <omp.h>


#define ADVANCE_FUDGE (0.015)		// should modify to calculate off the angle through intersection with the vector break


template<class _type>
ScenarioGridCache<_type>::ScenarioGridCache(CCWFGM_Scenario *scenario, const XY_Point &start_ll, const XY_Point& start_ur, const _type resolution /*, const std::uint16_t plot_X, const std::uint16_t plot_Y*/)
    : m_scenario(scenario), m_ll(start_ll), m_ur(start_ur), m_resolution(resolution) /*, m_plot_X(plot_X), m_plot_Y(plot_Y)*/ {
	m_resolution2 = m_resolution * m_resolution;
	m_resolution3 = m_resolution * m_resolution2;
	m_iresolution = 1.0 / m_resolution;
	m_iresolution2 = 1.0 / m_resolution2;
	m_iresolution3 = 1.0 / m_resolution3;

	m_breachAdvance = ADVANCE_FUDGE;
	gridToInternal1D(m_breachAdvance);

	m_minArea = 0.25;
	gridToInternal2D(m_minArea);
}


template<class _type>
bool ScenarioGridCache<_type>::allocCPArray() {

	return true;
}


template<class _type>
ScenarioGridCache<_type>::~ScenarioGridCache() {
}


template<class _type>
void ScenarioGridCache<_type>::Size(const XYPointType &ll, const XYPointType &ur) {
	if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_CACHE_GRID_POINTS))) {
		XYPointType dims = ur - ll;
		m_ll = ll;
		m_ur = ur;
		m_plot_X = (std::uint16_t)(dims.x * m_iresolution);
		m_plot_Y = (std::uint16_t)(dims.y * m_iresolution);
		return;
	}
}


template<class _type>
void ScenarioGridCache<_type>::grid_create(const XYPointType &ll, const XYPointType &ur) {
	m_ll = ll;
	m_ur = ur;
	XYPointType dims = ur - ll;
	m_plot_X = (std::uint16_t)(dims.x * m_iresolution);
	m_plot_Y = (std::uint16_t)(dims.y * m_iresolution);
}


template<class _type>
void ScenarioGridCache<_type>::grid_resize(const XYPointType &_ll, const XYPointType &_ur) {
	XYPointType ll(_ll), ur(_ur), dims = m_ll - ll;
	dims *= m_iresolution;
	dims.x = ceil(dims.x);
	dims.y = ceil(dims.y);
	if (dims.x < 0.0)	dims.x = 0.0;
	if (dims.y < 0.0)	dims.y = 0.0;		// won't support shrinking
	std::uint16_t off_x = (std::uint16_t)dims.x;
	std::uint16_t off_y = (std::uint16_t)dims.y;
	dims *= m_resolution;
	ll = m_ll;
	ll -= dims;

	dims = ur - m_ur;
	dims *= m_iresolution;
	dims.x = ceil(dims.x);
	dims.y = ceil(dims.y);
	if (dims.x < 0.0)	dims.x = 0.0;
	if (dims.y < 0.0)	dims.y = 0.0;		// won't support shrinking
	dims *= m_resolution;
	ur = m_ur;
	ur += dims;

	dims = ur - ll;
	dims *= m_iresolution;
	std::uint16_t plot_X = (std::uint16_t)dims.x;
	std::uint16_t plot_Y = (std::uint16_t)dims.y;

	if ((m_plot_X == plot_X) && (m_plot_Y == plot_Y))
		if (m_ll.Equals(ll) && (m_ur.Equals(ur)))
			return;						// bounding box grew larger but not enough to require a larger grid cache

	m_plot_X = plot_X;
	m_plot_Y = plot_Y;
	m_ll = ll;
	m_ur = ur;
}


template<class _type>
std::uint32_t ScenarioGridCache<_type>::arrayIndex(std::uint16_t x, std::uint16_t y) const {
	weak_assert(x < m_plot_X);
	weak_assert(y < m_plot_Y);
	return y * m_plot_X + x;
}


template<class _type>
std::uint32_t ScenarioGridCache<_type>::arrayIndex(const XYPointType &pt) const {
	XYPointType loc(pt);
	if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN))) {
		loc.x -= m_ll.x;
		loc.y -= m_ll.y;
	}
	if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
		loc.x *= m_iresolution;
		loc.y *= m_iresolution;
	}
	return arrayIndex((std::uint16_t)loc.x, (std::uint16_t)loc.y);
}


template<class _type>
_type ScenarioGridCache<_type>::resolution() const {
	return m_resolution;
}


template<class _type>
_type ScenarioGridCache<_type>::resolution2() const {
	return m_resolution2;
}


template<class _type>
_type ScenarioGridCache<_type>::iresolution() const {
	return m_iresolution;
}


template<class _type>
_type ScenarioGridCache<_type>::iresolution2() const {
	return m_iresolution2;
}


template<class _type>
_type ScenarioGridCache<_type>::iresolution3() const {
	return m_iresolution3;
}


template<class _type>
_type ScenarioGridCache<_type>::breachAdvance() const {
	return m_breachAdvance;
}


template<class _type>
_type ScenarioGridCache<_type>::minFireArea() const {
	return m_minArea;
}


template<class _type>
const typename ScenarioGridCache<_type>::XYPointType& ScenarioGridCache<_type>::start_ll() const {
	return m_ll;
}


template<class _type>
const typename ScenarioGridCache<_type>::XYPointType& ScenarioGridCache<_type>::start_center() const {
	return m_ll.PointBetween(m_ur);
}


template<class _type>
const typename ScenarioGridCache<_type>::XYPointType& ScenarioGridCache<_type>::start_ur() const {
	return m_ur;
}


template<class _type>
void ScenarioGridCache<_type>::toInternal(XY_Point& loc) const {
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN)) {
			loc.x -= m_ll.x;
			loc.y -= m_ll.y;
		}
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			loc.x *= m_iresolution;
			loc.y *= m_iresolution;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::toInternal(XYPointType& loc) const {
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN)) {
			loc.x -= m_ll.x;
			loc.y -= m_ll.y;
		}
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			loc.x *= m_iresolution;
			loc.y *= m_iresolution;
		}
	}
}
#endif

template<class _type>
void ScenarioGridCache<_type>::fromInternal(XY_Point& loc) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			loc.x *= m_resolution;
			loc.y *= m_resolution;
		}
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN)) {
			loc.x += m_ll.x;
			loc.y += m_ll.y;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::fromInternal(XYPointType& loc) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			loc.x *= m_resolution;
			loc.y *= m_resolution;
		}
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN)) {
			loc.x += m_ll.x;
			loc.y += m_ll.y;
		}
	}
}
#endif

template<class _type>
std::uint64_t ScenarioGridCache<_type>::toGridScaleX(const XY_Point& loc) const {
	double x = loc.x;
	x -= m_ll.x;
	x *= m_iresolution;
	return (std::uint64_t)x;
}


template<class _type>
std::uint64_t ScenarioGridCache<_type>::toGridScaleY(const XY_Point& loc) const {
	double y = loc.y;
	y -= m_ll.y;
	y *= m_iresolution;
	return (std::uint64_t)y;
}

#ifdef USE_BIGFLOATS
template<class _type>
std::uint64_t ScenarioGridCache<_type>::toGridScaleX(const XYPointType& loc) const {
	_type x = loc.x;
	x -= m_ll.x;
	x *= m_iresolution;
	return (std::uint64_t)x;
}


template<class _type>
std::uint64_t ScenarioGridCache<_type>::toGridScaleY(const XYPointType& loc) const {
	_type y = loc.y;
	y -= m_ll.y;
	y *= m_iresolution;
	return (std::uint64_t)y;
}
#endif

template<class _type>
void ScenarioGridCache<_type>::toInternal1D(double& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_iresolution;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::toInternal1D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_iresolution;
		}
	}
}
#endif

template<class _type>
void ScenarioGridCache<_type>::gridToInternal1D(double& value) const {
	if (m_scenario) {
		if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
			value *= m_resolution;
		}
	}
}

template<class _type>
void ScenarioGridCache<_type>::gridToInternal2D(double& value) const {
	if (m_scenario) {
		if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
			value *= m_resolution2;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::gridToInternal1D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
			value *= m_resolution;
		}
	}
}

template<class _type>
void ScenarioGridCache<_type>::gridToInternal2D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
			value *= m_resolution2;
		}
	}
}
#endif

template<class _type>
void ScenarioGridCache<_type>::toInternal2D(double& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_iresolution2;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::toInternal2D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_iresolution2;
		}
	}
}
#endif

template<class _type>
void ScenarioGridCache<_type>::toInternal3D(double& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_iresolution3;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::toInternal3D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_resolution3;
		}
	}
}
#endif

template<class _type>
void ScenarioGridCache<_type>::fromInternal1D(double& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_resolution;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::fromInternal1D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_resolution;
		}
	}
}
#endif


template<class _type>
void ScenarioGridCache<_type>::gridFromInternal1D(double& value) const {
	if (m_scenario) {
		if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
			value *= m_iresolution;
		}
	}
}

template<class _type>
void ScenarioGridCache<_type>::gridFromInternal2D(double& value) const {
	if (m_scenario) {
		if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
			value *= m_iresolution2;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::gridFromInternal1D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
			value *= m_iresolution;
		}
	}
}

template<class _type>
void ScenarioGridCache<_type>::gridFromInternal2D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))) {
			value *= m_iresolution2;
		}
	}
}
#endif

template<class _type>
void ScenarioGridCache<_type>::fromInternal2D(double& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_resolution2;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::fromInternal2D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_resolution2;
		}
	}
}
#endif

template<class _type>
void ScenarioGridCache<_type>::fromInternal3D(double& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_resolution3;
		}
	}
}

#ifdef USE_BIGFLOATS
template<class _type>
void ScenarioGridCache<_type>::fromInternal3D(_type& value) const {			// opportunity for optimization here with a direct call to fma
	if (m_scenario) {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
			value *= m_resolution3;
		}
	}
}
#endif

template<class _type>
void ScenarioGridCache<_type>::RecordTimeStep(ScenarioTimeStep<_type> *sts1) {
	if (!m_cpArray)
		return;

	if (m_scenario->m_optionFlags & (1ull <<  CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE))
		if (!sts1->m_displayable)
			return;			// no point in adding it if we're just going to delete it anyway

	CRWThreadSemaphoreEngage _semaphore_engageT(m_cplock, SEM_TRUE);
	const ScenarioTimeStep<_type> *sts = sts1, *sts_prev = sts->LN_Pred();

	if (!sts_prev->LN_Pred()) {
		return;
	}

	XYRectangleType bounds;
	ScenarioFire<_type> *sf = sts->m_fires.LH_Head();
	while (sf->LN_Succ()) {
		if (sf->BoundingBox(bounds)) {
			bounds.m_min -= m_ll;
			bounds.m_max -= m_ll;
			bounds.m_min *= m_iresolution;
			bounds.m_max *= m_iresolution;

			std::uint16_t x1 = (bounds.m_min.x < 0.0) ? 0 : (std::uint16_t)bounds.m_min.x,
				x2 = (bounds.m_max.x < 0.0) ? 0 : (std::uint16_t)bounds.m_max.x,
				y1 = (bounds.m_min.y < 0.0) ? 0 : (std::uint16_t)bounds.m_min.y,
				y2 = (bounds.m_max.y < 0.0) ? 0 : (std::uint16_t)bounds.m_max.y,
				x, y;
			if (x1 >= m_plot_X)	x1 = m_plot_X - 1;
			if (x2 >= m_plot_X)	x2 = m_plot_X - 1;
			if (y1 >= m_plot_Y)	y1 = m_plot_Y - 1;
			if (y2 >= m_plot_Y)	y2 = m_plot_Y - 1;

			for (x = x1; x <= x2; x++)
				for (y = y1; y <= y2; y++) {
					sts = sts1;
					std::uint32_t idx = arrayIndex(x, y);

					if (m_cpArray[idx].m_fp)
						continue;

					XYPointType pt(x, y);
					pt.x += 0.5;
					pt.y += 0.5;
					pt *= m_resolution;
					pt += m_ll;

					if (!sts->PointInArea(pt))
						continue;

					//				check to see if it's outide sf->prev, if not then skip
									// this check will be implicit due to how the simulation runs

					//				follow existing logic for picking curr or prev closest point and store

					FireFront<_type>* closest_ff;
				AGAIN:
					FirePoint<_type>* closest = sts->GetNearestPoint(pt, true, &closest_ff, true);

					if (!closest) {				// so, this will occur and will loop back until we find an appropriate closest point - shouldn't matter if the timestep includes all ScenarioFire's or not
						weak_assert(false);				// shouldn't occur due to the continue statement/test above
						sts = sts->LN_Pred();
						if (sts->LN_Pred())
							goto AGAIN;
						continue;
					}

					ScenarioFire<_type>* psf = closest_ff->Fire()->LN_CalcPred();
					if (psf)	sts_prev = psf->TimeStep();
					else		sts_prev = nullptr;

					FirePoint<_type>* prev_closest;
					FireFront<_type>* prev_closest_ff;
					if (sts_prev)	prev_closest = sts_prev->GetNearestPoint(pt, false, &prev_closest_ff, false);
					else			prev_closest = nullptr;

					if ((prev_closest) && (prev_closest->DistanceToSquared(pt) < closest->DistanceToSquared(pt))) {
						m_cpArray[idx].m_fp = prev_closest;
						m_cpArray[idx].m_ff = prev_closest_ff;
					}
					else {
						m_cpArray[idx].m_fp = closest;
						m_cpArray[idx].m_ff = closest_ff;
					}
					m_cpArray[idx].m_time = sts->m_time.GetTotalMicroSeconds();

				}
		}
		sf = sf->LN_Succ();
	}
}


template<class _type>
bool ScenarioGridCache<_type>::RetrieveCPoint(const XYPointType &pt, const WTime &time, bool displayable, FirePoint<_type> **fp, FireFront<_type> **ff) {
	if (!m_cpArray)
		return false;

	CRWThreadSemaphoreEngage _semaphore_engageT(m_cplock, SEM_FALSE);

	std::uint32_t idx = arrayIndex(pt);
	*fp = m_cpArray[idx].m_fp;
	*ff = m_cpArray[idx].m_ff;

	if (!displayable) {
		if (*ff)
			return ((!time.GetTime(0)) || (m_cpArray[idx].m_time <= time.GetTotalMicroSeconds()));
		return false;
	}

	// if we're limiting the search to displayable timesteps, then the rest of the code is called, and since the displayable timesteps
	// contain all ignitions that are active, we don't need to track them by LN_CalcPred(), just look for the prior displayable timestep

	const ScenarioTimeStep<_type> *sts, *sts_prev;
	if (*ff) {
		sts = (*ff)->Fire()->TimeStep();
		if (sts->m_time.GetTime(0) == m_cpArray[idx].m_time)
			sts_prev = sts->LN_Pred();
		else {
			sts_prev = sts;
			sts = sts->LN_Succ();
		}
	} else
		return false;

	while (!sts->m_displayable) {
		if ((time.GetTime(0)) && (sts->m_time > time))
			return false;
		sts = sts->LN_Succ();
		if (!sts->LN_Succ())
			return false;
	}

	if (sts_prev->LN_Pred()) {
		while (!sts_prev->m_displayable) {
			sts_prev = sts_prev->LN_Pred();
			if (!sts_prev->LN_Pred()) {
				sts_prev = nullptr;
				break;
			}
		}
	}

	FireFront<_type> *closest_ff;
	FirePoint<_type> *closest = sts->GetNearestPoint(pt, false, &closest_ff, true);

	FirePoint<_type> *prev_closest;
	FireFront<_type> *prev_closest_ff;
	if (sts_prev)	prev_closest = sts_prev->GetNearestPoint(pt, false, &prev_closest_ff, false);
	else			prev_closest = nullptr;

	if ((prev_closest) && (prev_closest->DistanceToSquared(pt) < closest->DistanceToSquared(pt))) {
		*fp = prev_closest;
		*ff = prev_closest_ff;
	} else {
		*fp = closest;
		*ff = closest_ff;
	}
	return true;
}


template<class _type>
bool ScenarioGridCache<_type>::RetrieveCPoint(const XYPointType& pt, const WTime& mintime, const WTime& time, bool displayable, FirePoint<_type>** fp, FireFront<_type>** ff) {
	if (!m_cpArray)
		return false;

	CRWThreadSemaphoreEngage _semaphore_engageT(m_cplock, SEM_FALSE);

	std::uint32_t idx = arrayIndex(pt);
	*fp = m_cpArray[idx].m_fp;
	*ff = m_cpArray[idx].m_ff;

	if (!displayable) {
		if (*ff)
			return ((!time.GetTime(0)) || ((m_cpArray[idx].m_time <= time.GetTotalMicroSeconds()) && (m_cpArray[idx].m_time > mintime.GetTotalMicroSeconds())));
		return false;
	}

	// if we're limiting the search to displayable timesteps, then the rest of the code is called, and since the displayable timesteps
	// contain all ignitions that are active, we don't need to track them by LN_CalcPred(), just look for the prior displayable timestep

	const ScenarioTimeStep<_type>* sts, * sts_prev;
	if (*ff) {
		sts = (*ff)->Fire()->TimeStep();
		if (sts->m_time.GetTotalMicroSeconds() < mintime.GetTotalMicroSeconds())
			return FALSE;
		if (sts->m_time.GetTime(0) == m_cpArray[idx].m_time)
			sts_prev = sts->LN_Pred();
		else {
			sts_prev = sts;
			sts = sts->LN_Succ();
		}
	}
	else
		return false;

	while (!sts->m_displayable) {
		if ((time.GetTime(0)) && (sts->m_time > time))
			return false;
		sts = sts->LN_Succ();
		if (!sts->LN_Succ())
			return false;
	}

	if (sts_prev->LN_Pred()) {
		while (!sts_prev->m_displayable) {
			sts_prev = sts_prev->LN_Pred();
			if (!sts_prev->LN_Pred()) {
				sts_prev = nullptr;
				break;
			}
		}
	}

	if ((sts_prev) && (sts_prev->m_time.GetTotalMicroSeconds() < mintime.GetTotalMicroSeconds()))
		sts_prev = nullptr;

	FireFront<_type>* closest_ff;
	FirePoint<_type>* closest = sts->GetNearestPoint(pt, false, &closest_ff, true);

	FirePoint<_type>* prev_closest;
	FireFront<_type>* prev_closest_ff;
	if (sts_prev)	prev_closest = sts_prev->GetNearestPoint(pt, false, &prev_closest_ff, false);
	else			prev_closest = nullptr;

	if ((prev_closest) && (prev_closest->DistanceToSquared(pt) < closest->DistanceToSquared(pt))) {
		*fp = prev_closest;
		*ff = prev_closest_ff;
	}
	else {
		*fp = closest;
		*ff = closest_ff;
	}
	return true;
}


template<class _type>
ScenarioCache<_type>::ScenarioCache(CCWFGM_Scenario* scenario, const XY_Point &start_ll, const XY_Point &start_ur, const _type resolution, const double landscapeFMC, const double landscapeElev, std::uint32_t numthreads) :
		ScenarioGridCache<_type>(scenario, start_ll, start_ur, resolution),
		m_numthreads(numthreads),
		m_specifiedFMC_Landscape(landscapeFMC),
		m_specifiedElev_Landscape(landscapeElev)
{
	m_pool = nullptr;
	m_multithread = (CWorkerThreadPool::NumberProcessors() > 1) && (numthreads >= 2);

	PreCalculation();

	m_assets = false;
	m_staticVectorBreaksLL = nullptr;				// turned off as per Cordy's instructions

	HRESULT hr;
	PolymorphicAttribute var;
	if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttribute(m_scenario->m_layerThread, CWFGM_GRID_ATTRIBUTE_SPATIALREFERENCE, &var))) {
		std::string projection;
		try { projection = std::get<std::string>(var); } catch (std::bad_variant_access &) { weak_assert(false); return; };

		m_coordinateConverter.SetSourceProjection(projection.c_str());
	}
}


template<class _type>
void ScenarioCache<_type>::InitThreadPool(bool multithread) {
	if (!m_pool) {
		m_multithread = (CWorkerThreadPool::NumberProcessors() > 1) && (multithread) && (m_numthreads >= 2);

		if (m_multithread) {
			m_pool = new CWorkerThreadPool(nullptr, nullptr, m_numthreads, THREAD_PRIORITY_BELOW_NORMAL, (m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY))) ? true : false);
			omp_set_num_threads(m_numthreads);
		}
		else {
			omp_set_num_threads(1);
		}
	}
	else if ((!multithread)) {
		delete m_pool;
		m_pool = nullptr;
		omp_set_num_threads(1);
	}
}


template<class _type>
ScenarioCache<_type>::~ScenarioCache() {
	if (m_staticVectorBreaksLL) {
		std::uint32_t i, cnt = (std::uint32_t)m_staticVectorBreaksLL->size();
		for (i = 0; i < cnt; i++)
			delete m_staticVectorBreaksLL->at(i);
		delete m_staticVectorBreaksLL;
	}
	AssetNode<_type>* an = m_scenario->m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		AssetGeometryNode<_type>* agn;
		while (agn = an->m_geometry.RemHead())
			delete agn;
		an = an->LN_Succ();
	}

	if (m_pool)
		delete m_pool;

	PostCalculation();
}


template<class _type>
void ScenarioCache<_type>::IgnitionExtents(XYRectangleType &bbox) {
	XYPointType pt;
	IgnitionNode<_type> *ignition = m_scenario->m_impl->m_ignitionList.LH_Head();
	while (ignition->LN_Succ()) {
		XY_Point min_pt, max_pt;
		if (SUCCEEDED(ignition->m_ignitionCOM->GetIgnitionRange((std::uint32_t)-1, &min_pt, &max_pt))) {
			pt.x = min_pt.x;
			pt.y = min_pt.y;
			if (ignition == m_scenario->m_impl->m_ignitionList.LH_Head()) {
				bbox.m_min = pt;
				bbox.m_max = pt;
			} else
				bbox.EncompassPoint(pt);
			pt.x = max_pt.x;
			pt.y = max_pt.y;
			bbox.EncompassPoint(pt);
		}
		ignition = ignition->LN_Succ();
	}
}


template<class _type>
void ScenarioCache<_type>::PreCalculation() {
	CalculationEventParms parms;

	XYRectangleType bbox;
	IgnitionExtents(bbox);

	parms.SimulationMin.x = bbox.m_min.x;
	parms.SimulationMin.y = bbox.m_min.y;
	parms.SimulationMax.x = bbox.m_max.x;
	parms.SimulationMax.y = bbox.m_max.y;

	m_scenario->m_gridEngine->PreCalculationEvent(m_scenario->m_layerThread, m_scenario->m_startTime, 0, &parms);

	VectorEngineNode *ven = m_scenario->m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->PreCalculationEvent(m_scenario->m_startTime, 0, &parms);
		ven = ven->LN_Succ();
	}
	AssetNode<_type>* an = m_scenario->m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->PreCalculationEvent(m_scenario->m_startTime, 0, &parms);
		an = an->LN_Succ();
	}
}


template<class _type>
void ScenarioCache<_type>::PostCalculation() {
	CalculationEventParms parms;
	AssetNode<_type>* an = m_scenario->m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->PostCalculationEvent(m_scenario->m_endTime, 0, &parms);
		an = an->LN_Succ();
	}

	VectorEngineNode *ven = m_scenario->m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->PostCalculationEvent(m_scenario->m_endTime, 0, &parms);
		ven = ven->LN_Succ();
	}

	m_scenario->m_gridEngine->PostCalculationEvent(m_scenario->m_layerThread, m_scenario->m_endTime, 0, &parms);
}


template<class _type>
bool cacheRetrieve(typename ScenarioCache<_type>::fuel_cache_key *entry, APTR lookup) {

	typename ScenarioCache<_type>::fuel_cache_retrieve *r = (typename ScenarioCache<_type>::fuel_cache_retrieve *)lookup;
	if (r->time != entry->time)
		return false;
	if (r->pt.x < entry->bbox.m_min.x)		return false;
	if (r->pt.x >= entry->bbox.m_max.x)		return false;
	if (r->pt.y < entry->bbox.m_min.y)		return false;
	if (r->pt.y >= entry->bbox.m_max.y)		return false;

	return true;
}


template<class _type>
ICWFGM_Fuel* ScenarioCache<_type>::GetFuel(const WTime& time, const XYZPointType& pt, bool& valid) const {
	XYPointType pt0(pt.x, pt.y);
	return GetFuel(time, pt0, valid);
}


#if defined(SUPPORT_BIGFLOATS) && defined(USE_BIGFLOATS)
template<class _type>
ICWFGM_Fuel* ScenarioCache<_type>::GetFuel(const WTime& time, const XY_Point& pt, bool& valid) const {
	XYPointType pt0(pt.x, pt.y); 
	return GetFuel(time, pt0, valid);
}
#endif


template<class _type>
ICWFGM_Fuel *ScenarioCache<_type>::GetFuel(const WTime &time, const XYPointType &pt, bool &valid) const {

	return GetFuel_NotCached(time, pt, valid);
}


template<class _type>
ICWFGM_Fuel* ScenarioCache<_type>::GetFuel_NotCached(const WTime& time, const XYPointType& pt, bool& valid) const {
	XY_Point _pt(pt);
	fromInternal(_pt);
	return GetFuelUTM_NotCached(time, _pt, valid);
}


template<class _type>
ICWFGM_Fuel* ScenarioCache<_type>::GetFuelUTM_NotCached(const WTime & time, const XYPointType& pt, bool& valid) const {
	HRESULT hr;
	ICWFGM_Fuel* fuel;
	bool fuel_valid;
	if (FAILED(hr = m_scenario->m_gridEngine->GetFuelData(m_scenario->m_layerThread, pt, time, &fuel, &fuel_valid, nullptr)) || (!fuel_valid)) {
		fuel = nullptr;
		valid = false;
	}
	else
		valid = fuel_valid;
	return fuel;}


template<class _type>
bool ScenarioCache<_type>::IsNonFuel_NotCached(const WTime &time, const XYPointType &pt, bool &valid, XYRectangleType *cache_bbox) const {
	XY_Point _pt(pt);
	fromInternal(_pt);
	return isNonFuelUTM_NotCached(time, _pt, valid, cache_bbox);
}


template<class _type>
bool ScenarioCache<_type>::isNonFuelUTM_NotCached(const WTime& time, const XYPointType& _pt, bool& valid, XYRectangleType* cache_bbox) const {
	HRESULT hr;
	bool result;
	ICWFGM_Fuel* fuel;
	XY_Rectangle bbox;
	hr = m_scenario->m_gridEngine->GetFuelData(m_scenario->m_layerThread, _pt, time, &fuel, &valid, (cache_bbox) ? &bbox : nullptr);
	if (cache_bbox)
		*cache_bbox = bbox;
	if (FAILED(hr))
		return true;
	else if (!valid)
		return true;
	else if (!fuel)
		return true;
	else if (FAILED(hr = fuel->IsNonFuel(&result)))
		return true;
	else if (result)
		return true;
	else
		return false;
}


template<class _type>
bool ScenarioCache<_type>::IsNonFuel(const WTime& time, const XYPointType& pt, bool& valid) const {
	XY_Point _pt(pt);
	fromInternal(_pt);

	return isNonFuelUTM_NotCached(time, _pt, valid, nullptr);

}


template<class _type>
bool ScenarioCache<_type>::IsNonFuelUTM(const WTime& time, const XYPointType& _pt, bool& valid) const {

	return isNonFuelUTM_NotCached(time, _pt, valid, nullptr);
}


template<class _type>
std::uint32_t ScenarioCache<_type>::AssetCount() const {
	return m_scenario->m_impl->m_assetList.GetCount();
}


template<class _type>
void ScenarioCache<_type>::buildStaticVectorBreaks() {
	if (m_staticVectorBreaksLL)
		return;

	m_staticVectorBreaksLL = new std::vector<XY_PolyLLSetBB<_type>*>();

	VectorEngineNode *ven = m_scenario->m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		std::uint32_t size, num, br_size, i;

		WTime zero((std::uint64_t)0, m_scenario->m_timeManager);
		if (SUCCEEDED(ven->m_vectorEngine->GetFireBreakCount(zero, &num)) && (num)) {
			if (SUCCEEDED(ven->m_vectorEngine->GetFireBreakSize((std::uint32_t)-1, (std::uint32_t)-1, zero, &size)) && (size)) {
				XY_Poly poly(0, XY_Poly::AllocStyle::ALLOCSTYLE_NEWDELETE);
				bool first = true;

				for (i = 0; i < num; i++) {
					std::uint32_t i_i, i_num;
					if (SUCCEEDED(ven->m_vectorEngine->GetFireBreakSetCount(zero, i, &i_num))) {
						for (i_i = 0; i_i < i_num; i_i++) {
							br_size = size;

							HRESULT hr = ven->m_vectorEngine->GetFireBreak(i, i_i, zero, &br_size, &poly);

							if (SUCCEEDED(hr) && (br_size)) {
								XY_PolyLLSetBB<_type>* poly_set = new XY_PolyLLSetBB<_type>();
								if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
									poly_set->SetCacheScale(resolution());

								XY_PolyLLTimed<_type>* poly_ll = poly_set->New();
								if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
									poly_ll->SetCacheScale(resolution());
								poly_ll->EnableCaching(false);
								for (std::uint32_t iii = 0; iii < poly.NumPoints(); iii++) {
									XY_PolyLLNode<_type>* n = poly_ll->New();
									auto p = poly.GetPoint(iii);
									toInternal(p);
									n->x = p.x;
									n->y = p.y;
									poly_ll->AddPoint(n);
								}

								poly_ll->m_publicFlags = XY_PolyLLTimed<_type>::Flags::INTERPRET_POLYGON;
								poly_ll->CleanPoly(0.0, XY_PolyLLTimed<_type>::Flags::INTERPRET_POLYGON);
								poly_ll->EnableCaching(true);
								std::int16_t rotation = poly_ll->DetermineRotation();
								if (rotation > 0)
									poly_ll->m_publicFlags |= XY_PolyLLTimed<_type>::Flags::INTERIOR_SPECIFIED;

								XYRectangleType bbox;
								if (poly_ll->BoundingBox(bbox)) {
									if (first) {
										poly_set->box = bbox;
										first = false;
									} else
										poly_set->box.EncompassRectangle(bbox);
								}
								poly_set->AddPoly(poly_ll);

								weak_assert(poly_set->NumPolys());
								poly_set->RescanRanges(false, m_multithread);
								m_staticVectorBreaksLL->push_back(poly_set);
							}
						}
					}
				}
			}
		}
		ven = ven->LN_Succ();
	}
}

template<class _type>
void ScenarioCache<_type>::buildAssets() {
	if (m_assets)
		return;

	m_assets = true;
	AssetNode<_type>* ven = m_scenario->m_impl->m_assetList.LH_Head();
	while (ven->LN_Succ()) {
		std::uint32_t size, num, br_size, i;

		XY_PolyLLSetBB<_type> poly_set;
		WTime zero((std::uint64_t)0, m_scenario->m_timeManager);
		if (SUCCEEDED(ven->m_asset->GetAssetCount(zero, &num)) && (num)) {
			if (SUCCEEDED(ven->m_asset->GetAssetSize((ULONG)-1, (ULONG)-1, zero, &size)) && (size)) {
				XY_Poly poly(0);

				bool first = true;

				for (i = 0; i < num; i++) {
					std::uint32_t i_i, i_num;
					if (SUCCEEDED(ven->m_asset->GetAssetSetCount(zero, i, &i_num))) {
						for (i_i = 0; i_i < i_num; i_i++) {
							br_size = size;

							std::uint16_t type;
							HRESULT hr = ven->m_asset->GetAsset(i, i_i, zero, &br_size, &poly, &type);

							if (SUCCEEDED(hr) && (br_size)) {
								AssetGeometryNode<_type>* agn = new AssetGeometryNode<_type>(m_scenario->m_timeManager);
								XYPolyLLType p_tmp;
								if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
									p_tmp.SetCacheScale(resolution());

								for (std::uint32_t iii = 0; iii < poly.NumPoints(); iii++) {
									XYPolyNodeType* n = p_tmp.New();
									auto p = poly.GetPoint(iii);
									toInternal(p);
									n->x = p.x;
									n->y = p.y;
									p_tmp.AddPoint(n);
								}

								XYPolyNodeType* n;
								while (n = p_tmp.RemHead())
									agn->m_geometry.AddTail(n);
								agn->m_geometry.m_publicFlags = type;

								weak_assert(type & XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_POLYMASK);

								if ((agn->m_geometry.IsPolygon())) { 
									agn->m_geometry.CleanPoly(0.0, XY_PolyLL::Flags::INTERPRET_POLYGON);

									SHORT rotation = agn->m_geometry.DetermineRotation();
									if (rotation > 0)
										agn->m_geometry.m_publicFlags |= XY_PolyLL::Flags::INTERIOR_SPECIFIED;
								}
								agn->m_geometry.RescanRanges(false);
								ven->m_geometry.AddTail(agn);
							}
						}
					}
				}
			}
		}
		ven = ven->LN_Succ();
	}
}


template<class _type>
bool ScenarioCache<_type>::CanBurn(const WTime &datetime, const XYPointType& centroid, const XYPointType &pt, const double rh, const double WindSpeed, const double fwi, const double isi) {
	WTimeSpan dayportion = datetime.GetTimeOfDay(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	WTimeSpan start, end;
	if (!CanBurnTime(datetime, centroid, start, end))
		return true;

	if (end > start) {
		if (dayportion < start) {
			WTimeSpan p_start, p_end;
			if (CanBurnTime(datetime - WTimeSpan(1, 0, 0, 0), centroid, p_start, p_end)) {
				if (p_end >= p_start) {
					if (p_end >= WTimeSpan(1, 0, 0, 0)) {
						p_end -= WTimeSpan(1, 0, 0, 0);
						if (dayportion > p_end)
							return true;
					}
				}
			}
			return false;
		}

		if ((dayportion.GetMinutes() == 59) && (dayportion.GetSeconds() == 59)) {	// important check for how we're handling hourly burning periods - another way to say this is we don't want
			if (dayportion > end)													// a timestep of 1 second, either - and for a full day we have to specify 23:59:59 and not 24:00:00
				return false;
		} else {
			if (dayportion >= end)													// check we need for handling burning periods to a per-sec like based on sun rise and set
				return false;
		}
	}
	else if (start != WTimeSpan(0L))
		return false;

	XY_Point _pt(pt);
	fromInternal(_pt);
	NumericVariant value;
	grid::AttributeValue value_valid;
	HRESULT hr;
	if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread, _pt, datetime, WTimeSpan(0), CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_MIN_RH, 0, &value, &value_valid, nullptr)) && (value_valid != grid::AttributeValue::NOT_SET)) {
		double min_rh;
		if (variantToDouble(value, &min_rh)) {
    		weak_assert(min_rh >= 0.0);
    		weak_assert(min_rh <= 1.0);
    		if (rh > min_rh)
    			return false;
		}
	}

	if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread, _pt, datetime, WTimeSpan(0), CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_MAX_WS, 0, &value, &value_valid, nullptr)) && (value_valid != grid::AttributeValue::NOT_SET)) {
		double max_ws;
		if (variantToDouble(value, &max_ws)) {
			if (WindSpeed < max_ws)
				return false;
		}
	}

	if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread, _pt, datetime, WTimeSpan(0), CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_MIN_FWI, 0, &value, &value_valid, nullptr)) && (value_valid != grid::AttributeValue::NOT_SET)) {
		double min_fwi;
		if (variantToDouble(value, &min_fwi)) {
			if (fwi < min_fwi)
				return false;
		}
	}

	if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread, _pt, datetime, WTimeSpan(0), CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_MIN_ISI, 0, &value, &value_valid, nullptr)) && (value_valid != grid::AttributeValue::NOT_SET)) {
		double min_isi;
		if (variantToDouble(value, &min_isi)) {
			if (isi < min_isi)
				return false;
		}
	}

	return true;
}


template<class _type>
bool ScenarioCache<_type>::CanBurn(const WTime &datetime, const XYPointType& centroid) {
	WTimeSpan dayportion = datetime.GetTimeOfDay(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	WTimeSpan start, end;
	if (!CanBurnTime(datetime, centroid, start, end))
		return true;

	if (end > start) {
		if (dayportion < start) {
			WTimeSpan p_start, p_end;
			if (CanBurnTime(datetime - WTimeSpan(1, 0, 0, 0), centroid, p_start, p_end)) {
				if (p_end >= p_start) {
					if (p_end >= WTimeSpan(1, 0, 0, 0)) {
						p_end -= WTimeSpan(1, 0, 0, 0);
						if (dayportion > p_end)
							return true;
					}
				}
			}
			return false;
		}

		if ((dayportion.GetMinutes() == 59) && (dayportion.GetSeconds() == 59)) {	// important check for how we're handling hourly burning periods - another way to say this is we don't want
			if (dayportion > end)													// a timestep of 1 second, either - and for a full day we have to specify 23:59:59 and not 24:00:00
				return false;
		}
		else {
			if (dayportion >= end)													// check we need for handling burning periods to a per-sec like based on sun rise and set
				return false;
		}
	}
	else if (start != WTimeSpan(0L))
		return false;

	return true;
}


template<class _type>
bool ScenarioCache<_type>::CanBurnTime(const WTime &dateTime, const XYPointType &centroid, WTimeSpan &start, WTimeSpan &end) {
	NumericVariant time;
	grid::AttributeValue time_valid;
	HRESULT hr;
	if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread, centroid, dateTime, WTimeSpan(0), CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_PERIOD_START_COMPUTED, 0, &time, &time_valid, nullptr)) &&
		(time_valid != grid::AttributeValue::NOT_SET)) {
		std::int64_t s_time;
		if (!(variantToInt64(time, &s_time))) {
			weak_assert(false);
			return false;
		}
		if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread, centroid, dateTime, WTimeSpan(0), CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_PERIOD_END_COMPUTED, 0, &time, &time_valid, nullptr)) &&
			(time_valid != grid::AttributeValue::NOT_SET)) {
			std::int64_t e_time;
			if (!(variantToInt64(time, &e_time))) {
				weak_assert(false);
				return false;
			}
			start = WTimeSpan(s_time);
			end = WTimeSpan(e_time);

			return true;
		}
	}
	return false;
}


template<class _type>
void ScenarioCache<_type>::GetCorrectedFuel(const XYZPointType& c_pt, const WTime& time, ICWFGM_Fuel* fuel, CCWFGM_FuelOverrides& overrides) {
	weak_assert(fuel);

	XYPointType _pt;
	_pt.x = c_pt.x;
	_pt.y = c_pt.y;
	return ScenarioCache<_type>::GetCorrectedFuel(_pt, time, fuel, overrides);
}


template<class _type>
void ScenarioCache<_type>::GetCorrectedFuel(const XYPointType &pt, const WTime& time, ICWFGM_Fuel* fuel, CCWFGM_FuelOverrides& overrides) {
	XY_Point _pt(pt);
	fromInternal(_pt);
	GetCorrectedFuelUTM(_pt, time, fuel, overrides);
}


template<class _type>
void ScenarioCache<_type>::GetCorrectedFuelUTM(const XYPointType& _pt, const WTime& time, ICWFGM_Fuel* fuel, CCWFGM_FuelOverrides& overrides) {
	HRESULT hr;
	bool is_mixed = false;
	if (SUCCEEDED(hr = fuel->IsMixedFuelType(&is_mixed)) && (is_mixed)) {
		NumericVariant v_pc;
		double pc1, pc2;
		grid::AttributeValue v_valid;
		if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
			_pt, time, WTimeSpan(0),
			FUELCOM_ATTRIBUTE_PC, m_scenario->m_optionFlags, &v_pc, &v_valid, nullptr)) && (v_valid != grid::AttributeValue::NOT_SET)) {
			variantToDouble(v_pc, &pc1);
			PolymorphicAttribute v_pc1;
			if (SUCCEEDED(fuel->GetAttribute(FUELCOM_ATTRIBUTE_PC, &v_pc1))) {
				VariantToDouble_(v_pc1, &pc2);
				pc1 *= 0.01;
				if (pc1 != pc2)
					overrides.AddOverride(FUELCOM_ATTRIBUTE_PC, pc1);
			}
		}			// we make a copy of the fuel type, then discard it because even though this would be a bit more expensive to do, we don't have to worry about
	}				// overriding the locking mechanism when we locked all fuel types to run the scenario, and we don't have to worry about any other issues
					// introduced when multithreading, or burning multiple scenarios simultaneously, or if it's a read-only fuel type anyway
	else if (SUCCEEDED(hr = fuel->IsMixedDeadFirFuelType(&is_mixed)) && (is_mixed)) {
		NumericVariant v_pc;
		double pc1, pc2;
		grid::AttributeValue v_valid;
		if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
			_pt, time, WTimeSpan(0),
			FUELCOM_ATTRIBUTE_PDF, m_scenario->m_optionFlags, &v_pc, &v_valid, nullptr)) && (v_valid != grid::AttributeValue::NOT_SET)) {
			variantToDouble(v_pc, &pc1);
			PolymorphicAttribute v_pc1;
			if (SUCCEEDED(fuel->GetAttribute(FUELCOM_ATTRIBUTE_PDF, &v_pc1))) {
				VariantToDouble_(v_pc1, &pc2);
				pc1 *= 0.01;
				if (pc1 != pc2)
					overrides.AddOverride(FUELCOM_ATTRIBUTE_PDF, pc1);
			}
		}
	}

	if (SUCCEEDED(hr = fuel->IsGrassFuelType(&is_mixed)) && (is_mixed)) {
		NumericVariant v_pc;
		double cure1, cure2;
		grid::AttributeValue v_valid;
		if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
			_pt, time, WTimeSpan(0),
			FUELCOM_ATTRIBUTE_CURINGDEGREE, m_scenario->m_optionFlags, &v_pc, &v_valid, nullptr)) && (v_valid != grid::AttributeValue::NOT_SET)) {
			variantToDouble(v_pc, &cure1);
			weak_assert(cure1 >= 0.0);
			weak_assert(cure1 <= 100.0);
			PolymorphicAttribute v_pc1;
			if (SUCCEEDED(fuel->GetAttribute(FUELCOM_ATTRIBUTE_CURINGDEGREE, &v_pc1))) {
				VariantToDouble_(v_pc1, &cure2);
				cure1 *= 0.01;
				if (cure1 != cure2)
					overrides.AddOverride(FUELCOM_ATTRIBUTE_CURINGDEGREE, cure1);
			}
		}

		if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
			_pt, time, WTimeSpan(0),
			FUELCOM_ATTRIBUTE_FUELLOAD, m_scenario->m_optionFlags, &v_pc, &v_valid, nullptr)) && (v_valid != grid::AttributeValue::NOT_SET)) {
			variantToDouble(v_pc, &cure1);
			PolymorphicAttribute v_pc1;
			if (SUCCEEDED(fuel->GetAttribute(FUELCOM_ATTRIBUTE_GFL, &v_pc1))) {
				VariantToDouble_(v_pc1, &cure2);
				if (cure1 != cure2)
					overrides.AddOverride(FUELCOM_ATTRIBUTE_GFL, cure1);
			}
		}
	}
	else {
		NumericVariant v_pc;
		double d1, d2;
		grid::AttributeValue v_valid;
		if (SUCCEEDED(hr = fuel->IsC6FuelType(&is_mixed)) && (is_mixed != false)) {
			if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
				_pt, time, WTimeSpan(0),
				FUELCOM_ATTRIBUTE_CBH, m_scenario->m_optionFlags, &v_pc, &v_valid, nullptr)) && (v_valid != grid::AttributeValue::NOT_SET)) {
				variantToDouble(v_pc, &d1);
				PolymorphicAttribute v_pc1;
				if (SUCCEEDED(fuel->GetAttribute(FUELCOM_ATTRIBUTE_CBH, &v_pc1))) {
					VariantToDouble_(v_pc1, &d2);
					if (d1 != d2)
						overrides.AddOverride(FUELCOM_ATTRIBUTE_CBH, d1);
				}
			}
		}

		if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
			_pt, time, WTimeSpan(0),
			FUELCOM_ATTRIBUTE_TREE_HEIGHT, m_scenario->m_optionFlags, &v_pc, &v_valid, nullptr)) && (v_valid != grid::AttributeValue::NOT_SET)) {
			variantToDouble(v_pc, &d1);
			PolymorphicAttribute v_pc1;
			if (SUCCEEDED(fuel->GetAttribute(FUELCOM_ATTRIBUTE_TREE_HEIGHT, &v_pc1))) {
				VariantToDouble_(v_pc1, &d2);
				if (d1 != d2)
					overrides.AddOverride(FUELCOM_ATTRIBUTE_TREE_HEIGHT, d1);
			}
		}

		if (SUCCEEDED(hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
			_pt, time, WTimeSpan(0),
			FUELCOM_ATTRIBUTE_FUELLOAD, m_scenario->m_optionFlags, &v_pc, &v_valid, nullptr)) && (v_valid != grid::AttributeValue::NOT_SET)) {
			variantToDouble(v_pc, &d1);
			PolymorphicAttribute v_pc1;
			if (SUCCEEDED(fuel->GetAttribute(FUELCOM_ATTRIBUTE_CFL, &v_pc1))) {
				VariantToDouble_(v_pc1, &d2);
				if (d1 != d2)
					overrides.AddOverride(FUELCOM_ATTRIBUTE_CFL, d1);
			}
		}
	}
}

#include "InstantiateClasses.cpp"
