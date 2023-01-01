/**
 * WISE_Scenario_Growth_Module: scenario.stats.cpp
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
#include "scenario.h"
#include "ScenarioTimeStep.h"
#include "results.h"
#include "FireEngine_ext.h"
#include "FuelCom_ext.h"


template<class _type>
HRESULT Scenario<_type>::GetVectorSize(std::uint32_t fire, WTime *time, std::uint32_t *size) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	ScenarioTimeStep<_type> *sts;
	HRESULT hr = GetStep(time, &sts, true);

	if (!sts) {
		*size = 0;
		return hr;
	}
	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);
	if (fire != (std::uint32_t)-1) {
		const FireFront<_type> *fs = sts->GetFireFront(fire);
		if (fs) {
			*size = fs->NumPoints();
			return S_OK;
		} else {
			*size = 0;
			return ERROR_SCENARIO_FIRE_UNKNOWN;
		}
	} else {
		*size = 0;
		const ScenarioFire<_type> *sf = sts->m_fires.LH_Head();
		while (sf->LN_Succ()) {
			const FireFront<_type> *fs = sf->LH_Head();
			while (fs->LN_Succ()) {
				if (*size < fs->NumPoints())
					*size = fs->NumPoints();
				fs = fs->LN_Succ();
			}
			sf = sf->LN_Succ();
		}
		return S_OK;
	}
}


template<class _type>
HRESULT Scenario<_type>::GetVectorArray(std::uint32_t fire, WTime *time, std::uint32_t *size, XY_Poly &xy_pairs) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	ScenarioTimeStep<_type> *sts;
	HRESULT hr = GetStep(time, &sts, true);

	if (!sts) {
		*size = 0;
		return hr;
	}
	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);
	const FireFront<_type> *fs = sts->GetFireFront(fire);
	if (fs) {
		if (xy_pairs.NumPoints() < fs->NumPoints())
			return E_OUTOFMEMORY;
		*size = fs->NumPoints();

		FirePoint<_type> *fp = fs->LH_Head();

		for (uint32_t i = 0; fp->LN_Succ(); fp = fp->LN_Succ(), i++) {
			xy_pairs.SetPoint(i, *fp);
		}
		return S_OK;
	} else {
		*size = 0;
		return ERROR_SCENARIO_FIRE_UNKNOWN;
	}
}


template<class _type>
HRESULT Scenario<_type>::GetStatsArray(const std::uint32_t fire, WTime *time, const std::uint16_t stat, std::uint32_t *size, std::vector<double> &stats) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	ScenarioTimeStep<_type> *sts;
	HRESULT hr = GetStep(time, &sts, true);

	if (!sts) {
		*size = 0;
		return hr;
	}
	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);
	const FireFront<_type> *fs = sts->GetFireFront(fire);
	if (fs) {
		if ((std::uint32_t)stats.size() < fs->NumPoints())
			return E_OUTOFMEMORY;
		*size = fs->NumPoints();

		FirePoint<_type> *fp = fs->LH_Head();
		for (std::uint32_t i = 0; fp->LN_Succ(); fp = fp->LN_Succ()) {
			double s;
			if (FAILED(hr = fp->RetrieveStat(stat, s)))
				return hr;
			stats[i++] = s;
		}
		return S_OK;
	} else {
		*size = 0;
		return ERROR_SCENARIO_FIRE_UNKNOWN;
	}
}


template<class _type>
HRESULT Scenario<_type>::GetStats(const std::uint32_t fire, ICWFGM_Fuel *fuel, WTime *time, const std::uint16_t stat, const std::uint16_t discretization, PolymorphicAttribute *stats) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	double tmp1, tmp2;
	WTimeSpan seconds;
	ScenarioTimeStep<_type> *sts, *prev_sts;
	HRESULT hr = GetStep(time, &sts, true);

	if (!sts) {
		PolymorphicAttribute s;
		*stats = s;
		return hr;
	}

	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);

	if (stat == CWFGM_FIRE_STAT_TIMESTEP_DURATION_SECS) {
		prev_sts = GetPreviousStep(sts, true, nullptr);
		if (prev_sts)
			*stats = sts->m_time - prev_sts->m_time;
		else
			*stats = WTimeSpan(0);
		return S_OK;
	} else if (stat == CWFGM_FIRE_STAT_TIMESTEP_MEMORY_BEGIN_USED) {
		*stats = sts->m_memoryBegin;
		return S_OK;
	} else if (stat == CWFGM_FIRE_STAT_TIMESTEP_MEMORY_END_USED) {
		*stats = sts->m_memoryEnd;
		return S_OK;
	} else if (stat == CWFGM_FIRE_STAT_TIMESTEP_TICKS) {
		*stats = (sts->m_tickCountEnd - sts->m_tickCountStart) / 10000;
		return S_OK;
	} else if (stat == CWFGM_FIRE_STAT_TIMESTEP_REALTIME) {
		auto value = sts->m_realtimeEnd - sts->m_realtimeStart;
		auto ms = std::chrono::duration_cast<std::chrono::microseconds>(value);
		*stats = (std::uint64_t)ms.count();
		return S_OK;
	} else if (stat == CWFGM_FIRE_STAT_TIMESTEP_CUMULATIVE_TICKS) {
		prev_sts = m_timeSteps.LH_Head();
		std::uint64_t s_stats = sts->m_tickCountEnd - sts->m_tickCountStart;
		while (prev_sts != sts) {
			if (prev_sts->m_displayable)
				s_stats += prev_sts->m_tickCountEnd - prev_sts->m_tickCountStart;
			prev_sts = prev_sts->LN_Succ();
		}
		*stats = (std::uint64_t)(s_stats / 10000);
		return S_OK;
	} else if (stat == CWFGM_FIRE_STAT_TIMESTEP_CUMULATIVE_REALTIME) {
		prev_sts = m_timeSteps.LH_Head();

#ifdef _NO_MFC
		auto value = sts->m_realtimeEnd - sts->m_realtimeStart;
		auto ms = std::chrono::duration_cast<std::chrono::microseconds>(value);
		double s_stats = ms.count();
		while (prev_sts != sts) {
			if (prev_sts->m_displayable) {
				value = prev_sts->m_realtimeEnd - prev_sts->m_realtimeStart;
				ms = std::chrono::duration_cast<std::chrono::microseconds>(value);
				s_stats += ms.count();
			}
			prev_sts = prev_sts->LN_Succ();
		}
		*stats = s_stats;
#else
		double s_stats = sts->m_realtimeEnd - sts->m_realtimeStart;
		while (prev_sts != sts) {
			if (prev_sts->m_displayable)
				s_stats += prev_sts->m_realtimeEnd - prev_sts->m_realtimeStart;
			prev_sts = prev_sts->LN_Succ();
		}
		*stats = s_stats;
#endif

		return S_OK;
	} else if (stat == CWFGM_FIRE_STAT_TIMESTEP_CUMULATIVE_BURNING_SECS) {
		double threshold = 1.0;
		toInternal2D(threshold);

		WTimeSpan cumulative(0);
		prev_sts = m_timeSteps.LH_Head();
		while (prev_sts != sts) {
			double dstats, prev_dstats;
			prev_sts->LN_Succ()->RetrieveStat(CWFGM_FIRE_STAT_AREA, &dstats);
			prev_sts->RetrieveStat(CWFGM_FIRE_STAT_AREA, &prev_dstats);
			if (dstats > (prev_dstats + threshold))
				cumulative += prev_sts->LN_Succ()->m_time - prev_sts->m_time;
			prev_sts = prev_sts->LN_Succ();
		}
		*stats = cumulative;
		return S_OK;
	} else if (stat == CWFGM_FIRE_STAT_NUM_TIMESTEPS) {
		*stats = m_timeSteps.NodeIndex(sts) + 1;
		return S_OK;
	} else if ((stat == CWFGM_FIRE_STAT_NUM_DISPLAY_TIMESTEPS) ||
		(stat == CWFGM_FIRE_STAT_NUM_EVENT_TIMESTEPS) ||
		(stat == CWFGM_FIRE_STAT_NUM_CALC_TIMESTEPS)) {
		std::uint32_t cnt = 0;
		ScenarioTimeStep<_type> *s = m_timeSteps.LH_Head();
		while (s->LN_Succ()) {
			if (((stat == CWFGM_FIRE_STAT_NUM_DISPLAY_TIMESTEPS) && (s->m_displayable)) ||
				((stat == CWFGM_FIRE_STAT_NUM_EVENT_TIMESTEPS) && (s->m_evented)) ||
				((stat == CWFGM_FIRE_STAT_NUM_CALC_TIMESTEPS) && (!s->m_evented) && (!s->m_displayable)))
				cnt++;
			if (s == sts)
				break;
			s = s->LN_Succ();
		}
		*stats = cnt;
		return S_OK;
	} else {
		switch (stat) {
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_INVOCATIONS:					*stats = (std::uint64_t)sts->m_advanceMetrics.numInvocations; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_INTERSECTIONS:				*stats = (std::uint64_t)sts->m_advanceMetrics.intersectionCount; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_UNIQUE_INTERSECTIONS:		*stats = (std::uint64_t)sts->m_advanceMetrics.uniqueIntersectionCount; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_FUDGED_INTERSECTIONS:		*stats = (std::uint64_t)sts->m_advanceMetrics.fudgedIntersectionCount; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_NEW_VERTEX_INTERSECTIONS:	*stats = (std::uint64_t)sts->m_advanceMetrics.newVertexIntersectionCount; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_POLYGON_START_COUNT:			*stats = (std::uint64_t)sts->m_advanceMetrics.polygonCountStart; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_POLYGON_LOGIC_REMOVED:		*stats = (std::uint64_t)sts->m_advanceMetrics.polygonCountLogicRemoved; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_POLYGON_TRIVIAL_REMOVED:		*stats = (std::uint64_t)sts->m_advanceMetrics.polygonCountTrivialRemoved; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_POLYGON_RETAINED:			*stats = (std::uint64_t)sts->m_advanceMetrics.polygonCountRetained; return S_OK;
		case CWFGM_FIRE_STAT_NUM_UNTANGLER_TICKS:						*stats = (std::uint64_t)sts->m_advanceMetrics.ticks / 10000; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_INVOCATIONS:					*stats = (std::uint64_t)sts->m_setMetrics.numInvocations; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_INTERSECTIONS:					*stats = (std::uint64_t)sts->m_setMetrics.intersectionCount; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_UNIQUE_INTERSECTIONS:			*stats = (std::uint64_t)sts->m_setMetrics.uniqueIntersectionCount; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_FUDGED_INTERSECTIONS:			*stats = (std::uint64_t)sts->m_setMetrics.fudgedIntersectionCount; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_NEW_VERTEX_INTERSECTIONS:		*stats = (std::uint64_t)sts->m_setMetrics.newVertexIntersectionCount; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_POLYGON_START_COUNT:			*stats = (std::uint64_t)sts->m_setMetrics.polygonCountStart; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_POLYGON_LOGIC_REMOVED:			*stats = (std::uint64_t)sts->m_setMetrics.polygonCountLogicRemoved; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_POLYGON_TRIVIAL_REMOVED:		*stats = (std::uint64_t)sts->m_setMetrics.polygonCountTrivialRemoved; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_POLYGON_RETAINED:				*stats = (std::uint64_t)sts->m_setMetrics.polygonCountRetained; return S_OK;
		case CWFGM_FIRE_STAT_NUM_POLYSET_TICKS:							*stats = (std::uint64_t)sts->m_setMetrics.ticks / 10000; return S_OK;
		
		case CWFGM_FIRE_STAT_ASSET_ARRIVAL_COUNT:						*stats = sts->m_assetCount; return S_OK;
		
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_INVOCATIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_INTERSECTIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_UNIQUE_INTERSECTIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_FUDGED_INTERSECTIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_NEW_VERTEX_INTERSECTIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_POLYGON_START_COUNT:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_POLYGON_LOGIC_REMOVED:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_POLYGON_TRIVIAL_REMOVED:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_POLYGON_RETAINED:
		case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_TICKS:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_INVOCATIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_INTERSECTIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_UNIQUE_INTERSECTIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_FUDGED_INTERSECTIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_NEW_VERTEX_INTERSECTIONS:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_POLYGON_START_COUNT:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_POLYGON_LOGIC_REMOVED:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_POLYGON_TRIVIAL_REMOVED:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_POLYGON_RETAINED:
		case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_TICKS:
		{
			std::uint64_t acc = 0;
			ScenarioTimeStep<_type> *s = m_timeSteps.LH_Head();
			while (s->LN_Succ()) {
				switch (stat) {
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_INVOCATIONS:				acc += s->m_advanceMetrics.numInvocations; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_INTERSECTIONS:			acc += s->m_advanceMetrics.intersectionCount; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_UNIQUE_INTERSECTIONS:		acc += s->m_advanceMetrics.uniqueIntersectionCount; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_FUDGED_INTERSECTIONS:		acc += s->m_advanceMetrics.fudgedIntersectionCount; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_NEW_VERTEX_INTERSECTIONS:	acc += s->m_advanceMetrics.newVertexIntersectionCount; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_POLYGON_START_COUNT:		acc += s->m_advanceMetrics.polygonCountStart; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_POLYGON_LOGIC_REMOVED:	acc += s->m_advanceMetrics.polygonCountLogicRemoved; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_POLYGON_TRIVIAL_REMOVED:	acc += s->m_advanceMetrics.polygonCountTrivialRemoved; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_POLYGON_RETAINED:			acc += s->m_advanceMetrics.polygonCountRetained; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_UNTANGLER_TICKS:					acc += s->m_advanceMetrics.ticks / 10000; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_INVOCATIONS:				acc += s->m_setMetrics.numInvocations; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_INTERSECTIONS:				acc += s->m_setMetrics.intersectionCount; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_UNIQUE_INTERSECTIONS:		acc += s->m_setMetrics.uniqueIntersectionCount; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_FUDGED_INTERSECTIONS:		acc += s->m_setMetrics.fudgedIntersectionCount; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_NEW_VERTEX_INTERSECTIONS:	acc += s->m_setMetrics.newVertexIntersectionCount; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_POLYGON_START_COUNT:		acc += s->m_setMetrics.polygonCountStart; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_POLYGON_LOGIC_REMOVED:		acc += s->m_setMetrics.polygonCountLogicRemoved; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_POLYGON_TRIVIAL_REMOVED:	acc += s->m_setMetrics.polygonCountTrivialRemoved; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_POLYGON_RETAINED:			acc += s->m_setMetrics.polygonCountRetained; break;
				case CWFGM_FIRE_STAT_CUMULATIVE_POLYSET_TICKS:						acc += s->m_setMetrics.ticks / 10000; break;
				}
				if (s == sts)
					break;
				s = s->LN_Succ();
			}
			*stats = acc;
			return S_OK;
		}
		}
	}

	if (fire != (std::uint32_t)-1) {
		const FireFront<_type> *fs = sts->GetFireFront(fire);
		double dstats;
		if (fs) {
			switch (stat) {
				case CWFGM_FIRE_STAT_ACTIVE_PERIMETER:		
				case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER:
				case CWFGM_FIRE_STAT_TOTAL_PERIMETER:		fs->RetrieveStat(stat, &dstats);
															fromInternal1D(dstats);
															*stats = dstats;
															return S_OK;

				case CWFGM_FIRE_STAT_AREA:					fs->RetrieveStat(stat, &dstats);
															fromInternal2D(dstats);
															weak_assert(dstats >= 0);
															if (dstats < 0)
																dstats = 0;
															*stats = dstats;
															return S_OK;

				case CWFGM_FIRE_STAT_NUM_POINTS:			*stats = fs->NumPoints();
															return S_OK;
				case CWFGM_FIRE_STAT_NUM_ACTIVE_POINTS:		*stats = fs->NumActivePoints();
															return S_OK;
				case CWFGM_FIRE_STAT_ROS:
				case CWFGM_FIRE_STAT_CFB:
				case CWFGM_FIRE_STAT_CFC:
				case CWFGM_FIRE_STAT_SFC:
				case CWFGM_FIRE_STAT_TFC:
				case CWFGM_FIRE_STAT_FI:
				case CWFGM_FIRE_STAT_HFI:
				case CWFGM_FIRE_STAT_HCFB:
				case CWFGM_FIRE_STAT_FLAMELENGTH:			fs->RetrieveStat(stat, &dstats);
															*stats = dstats;
															return S_OK;
			}
		} else {
			PolymorphicAttribute s;
			*stats = s;
			return ERROR_SCENARIO_FIRE_UNKNOWN;
		}
	} else {
		prev_sts = GetPreviousStep(sts, true, nullptr);
		double dstats;
		switch (stat) {
			case CWFGM_FIRE_STAT_ACTIVE_PERIMETER:
			case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER:
			case CWFGM_FIRE_STAT_TOTAL_PERIMETER:
															sts->RetrieveStat(stat, &dstats);
															fromInternal1D(dstats);
															*stats = dstats;
															return S_OK;

			case CWFGM_FIRE_STAT_AREA:						sts->RetrieveStat(stat, &dstats);
															fromInternal2D(dstats);
															*stats = dstats;
															return S_OK;

			case CWFGM_FIRE_STAT_ACTIVE_PERIMETER_GROWTH:
			case CWFGM_FIRE_STAT_ACTIVE_PERIMETER_CHANGE:
										sts->RetrieveStat(CWFGM_FIRE_STAT_ACTIVE_PERIMETER, &tmp1);
										if (prev_sts) {
											prev_sts->RetrieveStat(CWFGM_FIRE_STAT_ACTIVE_PERIMETER, &tmp2);
											seconds = sts->m_time - prev_sts->m_time;
										} else {
											tmp2 = 0.0;
											seconds = sts->m_time - m_scenario->m_startTime;
										}

										dstats = (tmp1 - tmp2);					// convert to per-meter
										if (stat == CWFGM_FIRE_STAT_ACTIVE_PERIMETER_GROWTH) {
											if (!seconds.GetTotalSeconds())
												dstats = 0.0;
											else	dstats = (dstats) * 60.0 / (double)seconds.GetTotalSeconds();	// convert to per-minute
										}
										*stats = dstats;
										return S_OK;
								
			case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER_GROWTH:
			case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER_CHANGE:
										sts->RetrieveStat(CWFGM_FIRE_STAT_EXTERIOR_PERIMETER, &tmp1);
										if (prev_sts) {
											prev_sts->RetrieveStat(CWFGM_FIRE_STAT_EXTERIOR_PERIMETER, &tmp2);
											seconds = sts->m_time - prev_sts->m_time;
										} else {
											tmp2 = 0.0;
											seconds = sts->m_time - m_scenario->m_startTime;
										}

										dstats = (tmp1 - tmp2);			// convert to per-meter
										fromInternal1D(dstats);
										if (stat == CWFGM_FIRE_STAT_EXTERIOR_PERIMETER_GROWTH) {
											if (!seconds.GetTotalSeconds())
												dstats = 0.0;
											else
												dstats = (dstats) * 60.0 / (double)seconds.GetTotalSeconds();	// convert to per-minute
										}
										*stats = dstats;
										return S_OK;
								
			case CWFGM_FIRE_STAT_TOTAL_PERIMETER_GROWTH:
			case CWFGM_FIRE_STAT_TOTAL_PERIMETER_CHANGE:
										sts->RetrieveStat(CWFGM_FIRE_STAT_TOTAL_PERIMETER, &tmp1);
										if (prev_sts) {
											prev_sts->RetrieveStat(CWFGM_FIRE_STAT_TOTAL_PERIMETER, &tmp2);
											seconds = sts->m_time - prev_sts->m_time;
										} else {
											tmp2 = 0.0;
											seconds = sts->m_time - m_scenario->m_startTime;
										}

										dstats = (tmp1 - tmp2);					// convert to per-meter
										fromInternal1D(dstats);
										if (stat == CWFGM_FIRE_STAT_TOTAL_PERIMETER_GROWTH) {
											if (!seconds.GetTotalSeconds())
												dstats = 0.0;
											else	dstats = (dstats) * 60.0 / (double)seconds.GetTotalSeconds();	// convert to per-minute
										}
										*stats = dstats;
										return S_OK;
								
			case CWFGM_FIRE_STAT_BURNED:
			case CWFGM_FIRE_STAT_BURNED_CHANGE:
			case CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED:
			case CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED:
			case CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED:
			case CWFGM_FIRE_STAT_RADIATIVE_POWER:
										sts->RetrieveStat(stat, fuel, discretization, (stat == CWFGM_FIRE_STAT_BURNED) ? nullptr : prev_sts, &dstats);
										*stats = dstats;
										return S_OK;
										// should just create a new function here, for this

			case CWFGM_FIRE_STAT_AREA_GROWTH:
			case CWFGM_FIRE_STAT_AREA_CHANGE:
										sts->RetrieveStat(CWFGM_FIRE_STAT_AREA, &tmp1);
										if (prev_sts) {
											prev_sts->RetrieveStat(CWFGM_FIRE_STAT_AREA, &tmp2);
											seconds = sts->m_time - prev_sts->m_time;
										} else {
											tmp2 = 0.0;
											seconds = sts->m_time - m_scenario->m_startTime;
										}

										dstats = (tmp1 - tmp2);			// convert to m2
										fromInternal2D(dstats);
										if (stat == CWFGM_FIRE_STAT_AREA_GROWTH) {
											if (!seconds.GetTotalSeconds())
												dstats = 0.0;
											else	dstats = (dstats) * 60.0 / (double)seconds.GetTotalSeconds();	// convert to per minute
										}
										*stats = dstats;
										return S_OK;

				case CWFGM_FIRE_STAT_CUMULATIVE_NUM_POINTS:
				case CWFGM_FIRE_STAT_CUMULATIVE_NUM_ACTIVE_POINTS:
				case CWFGM_FIRE_STAT_NUM_ACTIVE_POINTS:
				case CWFGM_FIRE_STAT_NUM_POINTS:
				case CWFGM_FIRE_STAT_NUM_ACTIVE_FRONTS:
				case CWFGM_FIRE_STAT_NUM_FRONTS:
										hr = sts->RetrieveStat(stat, &dstats);
										if (SUCCEEDED(hr)) {
											*stats = (uint32_t)dstats;
										} else {
											PolymorphicAttribute s;
											*stats = s;
										}
										return hr;
				case CWFGM_FIRE_STAT_ROS:
				case CWFGM_FIRE_STAT_CFB:			
				case CWFGM_FIRE_STAT_CFC:
				case CWFGM_FIRE_STAT_SFC:
				case CWFGM_FIRE_STAT_TFC:
				case CWFGM_FIRE_STAT_FI:
				case CWFGM_FIRE_STAT_HFI:
				case CWFGM_FIRE_STAT_HCFB:
				case CWFGM_FIRE_STAT_FLAMELENGTH:	
										hr = sts->RetrieveStat(stat, &dstats);
										if (SUCCEEDED(hr)) {
											*stats = dstats;
										} else {
											PolymorphicAttribute s;
											*stats = s;
										}
										return hr;

				case CWFGM_FIRE_STAT_SUNRISE:
				case CWFGM_FIRE_STAT_SUNSET:
				case CWFGM_FIRE_STAT_SOLARNOON:
				case CWFGM_FIRE_STAT_SUNRISE_CENTROID_X:
				case CWFGM_FIRE_STAT_SUNRISE_CENTROID_Y:
				case CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_PERIOD_START_COMPUTED:
				case CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_PERIOD_END_COMPUTED:
										hr = sts->RetrieveStat(stat, stats);
										return hr;
		}
	}

	PolymorphicAttribute s;
	*stats = s;
	return ERROR_FIRE_STAT_UNKNOWN;
}


template<class _type>
HRESULT Scenario<_type>::GetStats(const std::uint32_t fire, WTime *time, const std::uint16_t stat, const bool only_displayable, const double greater_equal, const double less_than, double *stats) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	WTimeSpan seconds;
	ScenarioTimeStep<_type> *sts;
	HRESULT hr = GetStep(time, &sts, only_displayable);

	if (!sts) {
		*stats = 0.0;
		return hr;
	}
	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);
	if (fire != (std::uint32_t)-1) {
		const FireFront<_type> *fs = sts->GetFireFront(fire);
		if (fs) {
			switch (stat) {
				case CWFGM_FIRE_STAT_FBP_RSI:
				case CWFGM_FIRE_STAT_FBP_ROSEQ:
				case CWFGM_FIRE_STAT_FBP_ROS:
				case CWFGM_FIRE_STAT_FBP_BROS:
				case CWFGM_FIRE_STAT_FBP_FROS:
				case CWFGM_FIRE_STAT_ROS:
				case CWFGM_FIRE_STAT_CFB:
				case CWFGM_FIRE_STAT_CFC:
				case CWFGM_FIRE_STAT_SFC:
				case CWFGM_FIRE_STAT_TFC:
				case CWFGM_FIRE_STAT_FI:
				case CWFGM_FIRE_STAT_HFI:
				case CWFGM_FIRE_STAT_HCFB:
				case CWFGM_FIRE_STAT_FLAMELENGTH:	
										fs->RetrieveStat(stat, greater_equal, less_than, stats);
										if (*stats > 0.0)
											*stats = *stats * 100.0 / (double)fs->NumActivePoints();
										return S_OK;
			}
		} else {
			*stats = 0;
			return ERROR_SCENARIO_FIRE_UNKNOWN;
		}
	} else {
		switch (stat) {
				case CWFGM_FIRE_STAT_FBP_RSI:
				case CWFGM_FIRE_STAT_FBP_ROSEQ:
				case CWFGM_FIRE_STAT_FBP_ROS:
				case CWFGM_FIRE_STAT_FBP_BROS:
				case CWFGM_FIRE_STAT_FBP_FROS:
				case CWFGM_FIRE_STAT_ROS:
				case CWFGM_FIRE_STAT_CFB:			
				case CWFGM_FIRE_STAT_CFC:
				case CWFGM_FIRE_STAT_SFC:
				case CWFGM_FIRE_STAT_TFC:
				case CWFGM_FIRE_STAT_FI:
				case CWFGM_FIRE_STAT_HFI:
				case CWFGM_FIRE_STAT_HCFB:
				case CWFGM_FIRE_STAT_FLAMELENGTH:
										sts->RetrieveStat(stat, greater_equal, less_than, stats);
										if (*stats > 0.0)
											*stats = *stats * 100.0 / (double)sts->NumActivePoints();
										return S_OK;
		}
	}
	*stats = 0;
	return ERROR_FIRE_STAT_UNKNOWN;
}


template<class _type>
HRESULT Scenario<_type>::getCalculatedStats(XYPointType c_pt, const WTime& time, ICWFGM_Fuel*& fuel, const CCWFGM_FuelOverrides &overrides, bool valid, std::uint64_t& flags, const std::uint32_t technique,
	double& fbp_rss, double& fbp_roseq, double& fbp_ros, double& fbp_fros, double& fbp_bros, double& fbp_raz, double* fbp_rosv, double* fbp_v,
	double* cfb, double* cfc, double* rso, double* csi, double* sfc, double* tfc, double* fi, double* flameLength, double* _fmc) {
	bool result = false;
	HRESULT hr;
	if ((!valid) || (!fuel)) {
		fbp_rss = fbp_roseq = fbp_ros = fbp_fros = fbp_bros = fbp_raz = 0.0;
		if (cfb)			*cfb = 0.0;
		if (cfc)			*cfc = 0.0;
		if (rso)			*rso = 0.0;
		if (csi)			*csi = 0.0;
		if (sfc)			*sfc = 0.0;
		if (tfc)			*tfc = 0.0;
		if (fi)				*fi = 0.0;
		if (flameLength)	*flameLength = 0.0;
		if (_fmc)			*_fmc = 0.0;
		if (fbp_rosv)		*fbp_rosv = 0.0;
		if (fbp_v)			*fbp_v = 0.0;
		return S_OK;
	}

	if ((FAILED(hr = fuel->IsNonFuel(&result))) || (result)) {
		fbp_rss = fbp_roseq = fbp_ros = fbp_fros = fbp_bros = fbp_raz = 0.0;
		if (cfb)			*cfb = 0.0;
		if (cfc)			*cfc = 0.0;
		if (rso)			*rso = 0.0;
		if (csi)			*csi = 0.0;
		if (sfc)			*sfc = 0.0;
		if (tfc)			*tfc = 0.0;
		if (fi)				*fi = 0.0;
		if (flameLength)	*flameLength = 0.0;
		if (_fmc)			*_fmc = 0.0;
		if (fbp_rosv)		*fbp_rosv = 0.0;
		if (fbp_v)			*fbp_v = 0.0;
		return S_OK;
	}

	double		aspect, azimuth;
	flags = m_scenario->m_optionFlags;
	XY_Point pt2;
	pt2.x = c_pt.x;
	pt2.y = c_pt.y;
	fromInternal(pt2);

	grid::TerrainValue elev_valid, terrain_valid;
	double _z;
	hr = m_scenario->m_gridEngine->GetElevationData(m_scenario->m_layerThread, pt2, true, &_z, &aspect, &azimuth, &elev_valid, &terrain_valid, nullptr);

	double fmc, ff;
	double latitude, longitude;

	if ((m_scenario->m_specifiedFMC >= 0.0) && (m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_SPECIFIED_FMC_ACTIVE))))
		fmc = m_scenario->m_specifiedFMC;
	else if (m_specifiedFMC_Landscape >= 0.0)
		fmc = m_specifiedFMC_Landscape;
	else {
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION)) {
			double x1, y1;
			NumericVariant nv;
			grid::AttributeValue av;
			hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
				pt2, time, WTimeSpan(0),
				CWFGM_FUELGRID_ATTRIBUTE_X_MID, 0, &nv, &av, nullptr);
			x1 = std::get<double>(nv);
			hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
				pt2, time, WTimeSpan(0),
				CWFGM_FUELGRID_ATTRIBUTE_Y_MID, 0, &nv, &av, nullptr);
			y1 = std::get<double>(nv);

			m_coordinateConverter.SourceToLatlon(1, &x1, &y1, nullptr);
			longitude = DEGREE_TO_RADIAN(x1);
			latitude = DEGREE_TO_RADIAN(y1);	
		} else {
			weak_assert(0);
			double x1 = m_coordinateConverter.xllcorner(), y1 = m_coordinateConverter.yllcorner();
			m_coordinateConverter.SourceToLatlon(1, &x1, &y1, nullptr);
			longitude = x1;
			latitude = y1;			
		}
		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FMC_TERRAIN)) {
			weak_assert(elev_valid != grid::TerrainValue::NOT_SET);
			double elev;
			if (elev_valid == grid::TerrainValue::DEFAULT) {
				if ((m_scenario->m_defaultElevation >= 0.0) || (m_scenario->m_defaultElevation == -99.0))
					elev = m_scenario->m_defaultElevation;
				else if ((m_specifiedElev_Landscape >= 0.0) || (m_specifiedElev_Landscape == -99.0))
					elev = m_specifiedElev_Landscape;
				else
					elev = _z;
			} else
				elev = _z;

			hr = fuel->FMC(latitude, longitude, elev, (std::uint16_t)time.GetDayOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST), &overrides, &fmc);
		} else {
			weak_assert(0);
			hr = fuel->FMC(latitude, longitude, -99.0, (std::uint16_t)time.GetDayOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST), &overrides, &fmc);
		}

		if (FAILED(hr)) {				// if we couldn't FMC for some reason...
			fbp_rss = fbp_roseq = fbp_ros = fbp_fros = fbp_bros = fbp_raz = 0.0;
			if (cfb)		*cfb = 0.0;
			if (cfc)		*cfc = 0.0;
			if (rso)		*rso = 0.0;
			if (csi)		*csi = 0.0;
			if (sfc)		*sfc = 0.0;
			if (tfc)		*tfc = 0.0;
			if (fi)			*fi = 0.0;
			if (flameLength)	*flameLength = 0.0;
			if (_fmc)		*_fmc = 0.0;
			if (fbp_rosv)		*fbp_rosv = 0.0;
			if (fbp_v)			*fbp_v = 0.0;
			return hr;
		}
	}
	if (_fmc)
		*_fmc = fmc;

	IWXData wx;
	IFWIData ifwi;
	DFWIData dfwi;
	bool wx_valid;
	hr = m_scenario->m_gridEngine->GetWeatherData(m_scenario->m_layerThread,
		    pt2, time,
  		    flags & ((1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMPORAL) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL)
		        | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_PRECIP) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND) | (1ull << (CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND_VECTOR))
				| (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMP_RH)
		        | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY)
				| (1ull << CWFGM_SCENARIO_OPTION_WEATHER_ALTERNATE_CACHE)) | (technique & (1ull << CWFGM_SCENARIO_OPTION_WEATHER_IGNORE_CACHE)),
			&wx, &ifwi, &dfwi, &wx_valid, nullptr);

	if (FAILED(hr) || (!wx_valid)) {					// if we couldn't get weather data, for any reason...
		fbp_rss = fbp_roseq = fbp_ros = fbp_fros = fbp_bros = fbp_raz = 0.0;
		if (cfb)		*cfb = 0.0;
		if (cfc)		*cfc = 0.0;
		if (rso)		*rso = 0.0;
		if (csi)		*csi = 0.0;
		if (sfc)		*sfc = 0.0;
		if (tfc)		*tfc = 0.0;
		if (fi)			*fi = 0.0;
		if (flameLength)	*flameLength = 0.0;
		if (_fmc)		*_fmc = 0.0;
		if (fbp_rosv)		*fbp_rosv = 0.0;
		if (fbp_v)			*fbp_v = 0.0;
		return hr;
	}

	hr = m_scenario->m_fwi->FF(ifwi.FFMC, time.GetTime(0), &ff);

	if (m_scenario->m_owd != -1.0)
		wx.WindDirection = DEGREE_TO_RADIAN(COMPASS_TO_CARTESIAN_DEGREE(m_scenario->m_owd));
	else if ((m_scenario->m_windTarget) && (m_scenario->m_windTargetIndex != (ULONG)-1) && (m_scenario->m_windTargetSubIndex != (ULONG)-1)) {
		XY_Point to;
		hr = m_scenario->m_windTarget->GetTarget(m_scenario->m_windTargetIndex, m_scenario->m_windTargetSubIndex, &to);
		if (SUCCEEDED(hr)) {
			toInternal(to);
			XY_Point from(floor(c_pt.x) + 0.5, floor(c_pt.y) + 0.5);
			wx.WindDirection = to.AngleTo(from);
		}
		else {
			fbp_rss = fbp_roseq = fbp_ros = fbp_fros = fbp_bros = fbp_raz = 0.0;
			if (cfb)			*cfb = 0.0;
			if (cfc)			*cfc = 0.0;
			if (rso)			*rso = 0.0;
			if (csi)			*csi = 0.0;
			if (sfc)			*sfc = 0.0;
			if (tfc)			*tfc = 0.0;
			if (fi)				*fi = 0.0;
			if (flameLength)	*flameLength = 0.0;
			if (_fmc)			*_fmc = 0.0;
			if (fbp_rosv)		*fbp_rosv = 0.0;
			if (fbp_v)			*fbp_v = 0.0;
			return hr;
		}
	}
	if (m_scenario->m_dwd != 0.0)
		wx.WindDirection -= DEGREE_TO_RADIAN(m_scenario->m_dwd);

	if (wx.Temperature < -50.0)	wx.Temperature = -50.0;
	else if (wx.Temperature > 60.0)	wx.Temperature = 60.0;

	if (wx.RH < 0.0)		wx.RH = 0.0;
	else if (wx.RH > 1.0)		wx.RH = 1.0;

	if (wx.WindSpeed > 200.0)	wx.WindSpeed = 200.0;
	else if (wx.WindSpeed < 0.0)	wx.WindSpeed = 0.0;

	double wsv, rsi, roseq, ros, frss, froseq, fros, brss, broseq, bros, raz;

		flags &= ~(1ull << CWFGM_SCENARIO_OPTION_ACCEL);
	NumericVariant greenup_on;
	grid::AttributeValue greenup_valid;
	hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
	    pt2, time, WTimeSpan(0),
	    CWFGM_SCENARIO_OPTION_GREENUP, 0, &greenup_on, &greenup_valid, nullptr);
	if (SUCCEEDED(hr) && (greenup_valid != grid::AttributeValue::NOT_SET)) {
		bool g_on;
		bool b = variantToBoolean(greenup_on, &g_on);
		if (b) {
			if (g_on)
				flags |= (1ull << CWFGM_SCENARIO_OPTION_GREENUP);
			else
				flags &= (~(1ull << CWFGM_SCENARIO_OPTION_GREENUP));
		}
	}

	bool grass;
	if (SUCCEEDED(fuel->IsGrassFuelType(&grass)) && (grass)) {
		NumericVariant standing_on;
		grid::AttributeValue standing_valid;
		hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
			pt2, time, WTimeSpan(0),
			CWFGM_SCENARIO_OPTION_GRASSPHENOLOGY, 0, &standing_on, &standing_valid, nullptr);
		if (SUCCEEDED(hr) && (standing_valid != grid::AttributeValue::NOT_SET)) {
			bool g_on;
			bool b = variantToBoolean(standing_on, &g_on);
			if (b) {
				if (g_on)
					flags |= (1ull << CWFGM_SCENARIO_OPTION_GRASSPHENOLOGY);
				else
					flags &= (~(1ull << CWFGM_SCENARIO_OPTION_GRASSPHENOLOGY));
			}
		}
	}

		double lb;
		hr = fuel->CalculateROSValues(aspect, azimuth, wx.WindSpeed, wx.WindDirection + CONSTANTS_NAMESPACE::Pi<double>(), dfwi.dBUI, fmc, ifwi.FFMC, ff, WTimeSpan(0), time.GetTimeOfDay(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST), (std::int16_t)(flags & 0xffff), &overrides,
			m_scenario, &rsi, &roseq, &ros, &frss, &froseq, &fros, &brss, &broseq, &bros, &lb, &wsv, &raz);
		fbp_rss = rsi;
		fbp_roseq = roseq;
		fbp_ros = ros;

		if ((ros > 0.0) && ((fbp_v) || (fbp_rosv))) {
			double fv, fvv;
			if (!fbp_v)			fbp_v = &fv;
			if (!fbp_rosv)		fbp_rosv = &fvv;
			fbp_bros = bros;
			fbp_fros = fros;
			fbp_raz = CARTESIAN_TO_COMPASS_RADIAN(raz);

			if (m_scenario->m_ovd != -1.0) {
				*fbp_v = m_scenario->m_ovd;
				hr = S_OK;
			} else {
				if ((m_scenario->m_vectorTarget) && (m_scenario->m_vectorTargetIndex != (ULONG)-1) && (m_scenario->m_vectorTargetSubIndex != (ULONG)-1)) {
					XY_Point to;
					hr = m_scenario->m_vectorTarget->GetTarget(m_scenario->m_vectorTargetIndex, m_scenario->m_vectorTargetSubIndex, &to);
					if (SUCCEEDED(hr)) {
						toInternal(to);
						XY_Point from(floor(c_pt.x) + 0.5, floor(c_pt.y) + 0.5);
						double cartesian = from.AngleTo(to);
						*fbp_v = RADIAN_TO_DEGREE(CARTESIAN_TO_COMPASS_RADIAN(cartesian));
					} else {
						*fbp_rosv = -1.0;
						*fbp_v = -1.0;
						hr = ERROR_INVALID_DATA | ERROR_SEVERITY_WARNING;	// neither the override, nor a grid value is provided, so we can't do anything
					}
				} else {
					NumericVariant vector;
					grid::AttributeValue vector_valid;

					hr = m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_layerThread,
						c_pt, time, WTimeSpan(0),
						CWFGM_SCENARIO_OPTION_GRID_OVD, m_scenario->m_optionFlags, &vector, &vector_valid, nullptr);
					if (SUCCEEDED(hr) && (vector_valid != grid::AttributeValue::NOT_SET)) {
						*fbp_v = std::get<double>(vector);
					} else {
						*fbp_rosv = -1.0;
						*fbp_v = -1.0;
						hr = ERROR_INVALID_DATA | ERROR_SEVERITY_WARNING;	// neither the override, nor a grid value is provided, so we can't do anything
					}
				}
			}
			if (hr != (ERROR_INVALID_DATA | ERROR_SEVERITY_WARNING)) {
				*fbp_v = DEGREE_TO_RADIAN(*fbp_v);
				if (m_scenario->m_dvd != 0.0)
					(*fbp_v) -= DEGREE_TO_RADIAN(m_scenario->m_dvd);

				hr = fuel->CalculateROSTheta(roseq, froseq, broseq, fbp_raz, *fbp_v, fbp_rosv);
			}
		} else {
			fbp_bros = 0.0;
			fbp_fros = 0.0;
			fbp_raz = 0.0;
			if (fbp_rosv)
				*fbp_rosv = 0.0;
			if (fbp_v)
				*fbp_v = 0.0;
		}

		if (cfb) {
			fuel->CalculateFCValues(ifwi.FFMC, dfwi.dBUI, fmc, rsi, ros, (std::int16_t)(flags & 0xffff), &overrides, cfb, cfc, rso, csi, sfc, tfc, fi);
			if ((fi) && (flameLength))
				*flameLength = FirePoint<_type>::flameLength(fuel, *cfb, *fi, &overrides);
		}
	return hr;
}


#include "InstantiateClasses.cpp"
