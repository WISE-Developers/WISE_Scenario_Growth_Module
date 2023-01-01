/**
 * WISE_Scenario_Growth_Module: scenario.cpp
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

#if __has_include(<mathimf.h>)
#include <mathimf.h>
#else
#include <cmath>
#endif

#include <cpl_string.h>
#include "scenario.h"
#include "ScenarioTimeStep.h"
#include "results.h"
#include "FireEngine_ext.h"
#include "GridCom_ext.h"
#include "WeatherCom_ext.h"
#include "convert.h"
#include "macros.h"
#include "excel_tinv.h"
#include "gdalclient.h"
#include "CWFGM_Scenario_Internal.h"
#include <omp.h>

#ifdef __GNUC__
#define BOOST_CHRONO_HEADER_ONLY
#endif
#include "boost_ll_config.h"
#include <boost/chrono.hpp>

#ifdef _MSC_VER
#undef _NO_MFC
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <psapi.h>
#else
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#endif


IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(ActiveFire, ActiveFire, 128 * 1024 / sizeof(ActiveFire<fireengine_float_type>), false, 16, fireengine_float_type)
IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(Scenario, Scenario, 10 * 1024 * 1024 / sizeof(Scenario<fireengine_float_type>), false, 16, fireengine_float_type)
IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(IgnitionNode, IgnitionNode, 128 * 1024 / sizeof(IgnitionNode<fireengine_float_type>), false, 16, fireengine_float_type)
IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(AssetNode, AssetNode, 128 * 1024 / sizeof(AssetNode<fireengine_float_type>), false, 16, fireengine_float_type)
IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(AssetGeometryNode, AssetGeometryNode, 128 * 4 * 1024 / sizeof(AssetGeometryNode<fireengine_float_type>), false, 16, fireengine_float_type)


template<class _type>
ActiveFire<_type>::ActiveFire() :
	m_startTime(0, nullptr),
	m_endTime(0, nullptr) {
	m_mate_next = m_mate_prev = this;
	m_master = nullptr;
}


template<class _type>
ActiveFire<_type>::ActiveFire(ActiveFire<_type> *master) :
	m_startTime(master->m_startTime),
	m_endTime(master->m_endTime),
	m_master(master),
	m_boundingBox(master->m_boundingBox),
	m_advanced(master->m_advanced) {
	RefNode<ScenarioFire<_type>>::LN_Ptr(master->LN_Ptr());
	m_mate_next = m_mate_prev = this;
}


template<class _type>
void ActiveFire<_type>::Attach(ActiveFire<_type> *mate) {
	m_mate_next->m_mate_prev = mate->m_mate_prev;
	mate->m_mate_prev->m_mate_next = m_mate_next;
	mate->m_mate_prev = this;
	m_mate_next = mate;
}


template<class _type>
bool ActiveFire<_type>::Attached(ActiveFire<_type> *mate) const
{
	ActiveFire<_type> *f = m_mate_next;
	while (f != this)
	{
		if (f == mate)
			return true;
		f = f->m_mate_next;
	}
	return false;
}


template<class _type>
void ActiveFire<_type>::Detach() {
	if (m_mate_next != this) {
		m_mate_prev->m_mate_next = m_mate_next;
		m_mate_next->m_mate_prev = m_mate_prev;
		m_mate_prev = m_mate_next = this;
	}
}


template<class _type>
Scenario<_type>::Scenario(CCWFGM_Scenario *scenario, const XY_Point &start_ll, const XY_Point &start_ur, const _type resolution, const double landscapeFMC, const double landscapeElev)
    : ScenarioCache<_type>(scenario, start_ll, start_ur, resolution, landscapeFMC, landscapeElev, scenario->m_threadingNumProcessors),
    m_closestcache(scenario->m_threadingNumProcessors << 1) {
	m_stepState = S_OK;

	m_omp_gvs_array = nullptr;
	m_omp_gps_array = nullptr;
	m_omp_gvs_array_size = 0;
	m_omp_gps_array_size = 0;

	if (scenario->m_growthPercentile >= 0.0) {
		m_tinv = tinv(scenario->m_growthPercentile / 100.0, 9999999);
	}
}


template<class _type>
Scenario<_type>::~Scenario() {
	ScenarioTimeStep<_type> *sts;
	while (sts = (ScenarioTimeStep<_type> *)m_timeSteps.RemHead())
		delete sts;
	ActiveFire<_type> *af;
	while (af = m_activeFires.RemHead()) {
		af->Detach();
		delete af;
	}

	if (m_omp_gvs_array)	free(m_omp_gvs_array);
	if (m_omp_gps_array)	free(m_omp_gps_array);
}


template<class _type>
std::uint64_t Scenario<_type>::GetProcessTickCount() {
	auto time = boost::chrono::process_user_cpu_clock::now();
	return time.time_since_epoch().count();
}


template<class _type>
HRESULT Scenario<_type>::Step() {

	CRWThreadSemaphoreEngage _semaphore_engageS(m_stepLock, SEM_TRUE);

	XYRectangleType bbox;

	if (m_timeSteps.GetCount()) {
		ScenarioTimeStep<_type> *sts = m_timeSteps.LH_Tail();
		if (sts->m_time == m_scenario->m_endTime) {
			m_stepState = SUCCESS_SCENARIO_SIMULATION_COMPLETE;
			return SUCCESS_SCENARIO_SIMULATION_COMPLETE;
		}

		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_BOUNDARY_STOP)) {
			PolymorphicAttribute v;
			XYPointType ll, ur;
			m_scenario->m_gridEngine->GetAttribute(m_scenario->m_layerThread, CWFGM_GRID_ATTRIBUTE_XLLCORNER, &v);
			try { 
				ll.x = static_cast<_type>(std::get<double>(v));
			} catch (std::bad_variant_access &) { 
				weak_assert(0);
				return ERROR_SCENARIO_BAD_STATE;
			};

			m_scenario->m_gridEngine->GetAttribute(m_scenario->m_layerThread, CWFGM_GRID_ATTRIBUTE_YLLCORNER, &v);
			try { 
				ll.y = static_cast<_type>(std::get<double>(v));
			} catch (std::bad_variant_access &) { 
				weak_assert(0);
				return ERROR_SCENARIO_BAD_STATE;
			};

			m_scenario->m_gridEngine->GetAttribute(m_scenario->m_layerThread, CWFGM_GRID_ATTRIBUTE_XURCORNER, &v);
			try { 
				ur.x = static_cast<_type>(std::get<double>(v));
			} catch (std::bad_variant_access &) { 
				weak_assert(0);
				return ERROR_SCENARIO_BAD_STATE;
			};

			m_scenario->m_gridEngine->GetAttribute(m_scenario->m_layerThread, CWFGM_GRID_ATTRIBUTE_YURCORNER, &v);
			try { 
				ur.y = static_cast<_type>(std::get<double>(v));
			} catch (std::bad_variant_access &) { 
				weak_assert(0);
				return ERROR_SCENARIO_BAD_STATE;
			};

			if (sts->BoundingBox(bbox)) {			// check to see if we've hit the boundary and we were told to do so when that happened.
				fromInternal(bbox.m_min);
				fromInternal(bbox.m_max);
				if ((bbox.m_min.x <= ll.x) || (bbox.m_min.y <= ll.y) || (bbox.m_max.x >= ur.x) || (bbox.m_max.y >= ur.y)) {
					m_stepState = SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS;
					return SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS;
				}
			}
		}
	}

	HRESULT retval = S_OK;
	std::uint64_t tickStart = GetProcessTickCount();

	std::chrono::time_point<std::chrono::system_clock> oleStart = std::chrono::system_clock::now();

	if (!ScenarioCache<_type>::StaticVectorBreak())
		ScenarioCache<_type>::buildStaticVectorBreaks();
	ScenarioCache<_type>::buildAssets();

	ScenarioTimeStep<_type>*sts = m_timeSteps.LH_Tail();

	WTime step_completion((std::uint64_t)0, m_scenario->m_timeManager);
	if (sts->LN_Pred()) {
		if (m_scenario->m_displayInterval.GetTotalSeconds()) {
			step_completion = sts->m_time + m_scenario->m_displayInterval;
			if (m_scenario->m_endTime < step_completion)
				step_completion = m_scenario->m_endTime;
		} else	step_completion = m_scenario->m_endTime;
	} else {
		step_completion = m_scenario->m_startTime;		// step_completion now has the time at which this step will be at when it's done
	}

	// https://stackoverflow.com/questions/24862488/thread-affinity-with-windows-msvc-and-openmp
	//set the thread affinity and priority
	if (m_timeSteps.IsEmpty()) {
		if (ScenarioCache<_type>::m_multithread) {
			int thread_id = (m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY))) ? -1 : -2;
			#pragma omp parallel for num_threads(ScenarioGridCache<_type>::m_scenario->m_threadingNumProcessors) firstprivate(thread_id)
			for (int i = 0; i < (m_scenario->m_threadingNumProcessors); i++)
			{
#ifdef _MSC_VER
				if (thread_id == -1) {
					thread_id = omp_get_thread_num();
					CWorkerThread::native_handle_type thread = CWorkerThreadPool::GetCurrentThread();
					CWorkerThreadPool::SetThreadAffinityToMask(thread, thread_id);
					SetThreadPriority(thread, THREAD_PRIORITY_BELOW_NORMAL);
				}
#else
				if (ScenarioGridCache<_type>::m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY)))
				{
					//set the affinity for the current thread
					cpu_set_t cpuset;
					CPU_ZERO(&cpuset);
					CPU_SET(omp_get_thread_num(), &cpuset);
					pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
				}

				//lookup the minimum possible priority for the current scheduling policy
				pthread_attr_t attr;
				int policy = 0;
				pthread_attr_init(&attr);
				pthread_attr_getschedpolicy(&attr, &policy);
				int lowestPrio = sched_get_priority_min(policy);
				pthread_attr_destroy(&attr);

				//set the thread priority to the lowest possible value
				pthread_setschedprio(pthread_self(), lowestPrio);
#endif
			}
		}
	}

	sts = nullptr;
	do {

#ifndef _NO_MFC
		PROCESS_MEMORY_COUNTERS_EX ppm;
		HANDLE proc = GetCurrentProcess();
		BOOL result = GetProcessMemoryInfo(proc, (PPROCESS_MEMORY_COUNTERS)&ppm, sizeof(ppm));
		DWORD error = GetLastError();
		std::uint64_t used = ppm.PagefileUsage;

		MEMORYSTATUSEX memory;
		memory.dwLength = sizeof(memory);
		::GlobalMemoryStatusEx(&memory);
#endif

#ifndef _MSC_VER
		struct rusage usage;
		getrusage(RUSAGE_SELF, &usage);
		std::uint64_t used = usage.ru_maxrss * 1024;
#endif

		std::uint64_t buffer = 512 * 1024 * 1024;			// make sure we have at least 500MB free to gracefully report to the user, shutdown or clean up
		if (m_timeSteps.GetCount()) {
			std::uint64_t buffer2 = m_timeSteps.LH_Tail()->m_memoryEnd - m_timeSteps.LH_Tail()->m_memoryBegin;
			if (((std::int64_t)buffer2) > 0) {
				buffer2 = buffer2 << 1;
				if (buffer2 > buffer)
					buffer = buffer2;
			}
		}

#ifndef _NO_MFC
		if (buffer > memory.ullAvailPageFile) {
			retval = E_OUTOFMEMORY;				// likely not enough memory so fail gracefully
			break;
		}
#endif

		if (sts)
			sts->m_lock.Unlock();

		m_llLock.Lock_Write();
		sts = new ScenarioTimeStep<_type>(this, step_completion, (step_completion == m_scenario->m_endTime));	// this appends itself to the list of time steps and calculates what it's time
		m_llLock.Unlock();

#if (!defined(_NO_MFC)) || (!defined(_MSC_VER))
		sts->m_memoryBegin = used;				// should be, all automatically
#endif

		bool advanced = sts->AdvanceFires();			// this copies all the fires from the previous state, then "grows" them based on
														// the previous state's statics for each fire point - this has to do the smoothing
														// that Cordy wants

		if ((advanced) && ((m_scenario->m_perimeterSpacing != 0.0)))
			sts->SimplifyFires();
		else
			sts->SimplifyFiresNull();

		if (advanced)
			sts->TrackFires();				// this pulls back any points due to intersections with fire breaks based on paths
		else
			sts->TrackFiresNull();

		sts->UnWindFires(advanced);				// this deals with any loops or knots, and may add or remove from the set of fires

		advanced |= sts->AddIgnitions();			// this adds any new ignitions to the fold, as necessary - it also make sure that
									// the new ignitions can actually burn and works from there - it also (necessarily)
									// unwinds them into usable fire fronts (before clipping them against any breaks)

		if (advanced)
			sts->UnOverlapFires();				// this deals with fires contacting each other, overlapped areas are given to the
									// the fire with the larger area from the previous step - new ignitions have no
									// area from the previous step
		else
			sts->UnOverlapFiresNull();

		if (advanced)
			sts->AddFirePoints();				// this introduces new fire points as we need them, based on m_perimeterResolution

		sts->StatsFires();					// this calculates FBP values, then Gwyn's equations for full statics on every
											// (active) fire vertex

		ScenarioFire<_type> *sf = sts->m_fires.LH_Head();		// the other routines above may have actually completely eliminated all fire fronts from a given
		while (sf->LN_Succ()) {					// ignition - this loop simply does some housekeeping to clean things up (and make sure that the
			if (sf->NumPolys()) {				// other housekeeping already performed was done correctly)
				sf->RescanRanges(false, ScenarioCache<_type>::m_multithread ? true : false);

				FireFront<_type> *ff = sf->LH_Head();
				while (ff->LN_Succ()) {
					weak_assert(ff->NumPoints() >= 3);
					ff = ff->LN_Succ();
				}
				sf = sf->LN_Succ();
			} else {
				ActiveFire<_type> *af = m_activeFires.LH_Head();
				while (af->LN_Succ()) {
					if (af->LN_Ptr() == sf) {
						af->LN_Ptr(nullptr);
					}
					af = af->LN_Succ();
				}
				ScenarioFire<_type> *sf1 = sf->LN_Succ();
				sts->m_fires.Remove(sf);
				delete sf;
				sf = sf1;
			}
		}

		sts->PostCalculation();
		m_closestcache.Clear();

		RecordTimeStep(sts);

		bool make_displayable = false;
		if (sts->CheckAssets(make_displayable)) {
			retval = SUCCESS_SCENARIO_SIMULATION_COMPLETE_ASSET;
			sts->m_displayable = 1;
			break;
		}
		if (make_displayable)
			sts->m_displayable = 1;

		HRESULT condition;
		if (sts->CheckStops(condition)) {
			retval = condition;
			sts->m_displayable = 1;
			break;
		}

		if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_BOUNDARY_STOP)) {
			if (sts->BoundingBox(bbox)) {				// check to see if we've hit the boundary and we were told to do so when that happened.
				fromInternal(bbox.m_min);
				fromInternal(bbox.m_max);
				if ((bbox.m_min.x <= sts->current_ll().x) ||
					(bbox.m_min.y <= sts->current_ll().y) ||
					(bbox.m_max.x >= sts->current_ur().x) ||
					(bbox.m_max.y >= sts->current_ur().y)) {

					retval = SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS;
					sts->m_displayable = 1;
					break;
				}
			}
		}

#ifndef _NO_MFC
		result = GetProcessMemoryInfo(proc, (PPROCESS_MEMORY_COUNTERS)&ppm, sizeof(ppm));
		error = GetLastError();
		sts->m_memoryEnd = ppm.PagefileUsage;		// should be, all automatically
#endif

#ifndef _MSC_VER
		getrusage(RUSAGE_SELF, &usage);
		sts->m_memoryEnd = usage.ru_maxrss * 1024;
#endif

	} while ((sts->m_time < step_completion) && (m_scenario->m_displayInterval.GetTotalSeconds()));

	if ((sts) && (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE))) {
		m_llLock.Lock_Write();

		ScenarioTimeStep<_type> *sts2 = Purge();

		m_llLock.Unlock();
	}

	if (sts) {
		weak_assert(sts->m_displayable == 1);				// the last one in a step is always the displayable one
		sts->RecordActiveFires();
		sts->m_tickCountStart = tickStart;
		sts->m_realtimeStart = oleStart;
		sts->m_realtimeEnd = std::chrono::system_clock::now();
		sts->m_tickCountEnd = GetProcessTickCount();
		sts->m_lock.Unlock();
		if (sts->m_time == m_scenario->m_endTime) {
			retval = SUCCESS_SCENARIO_SIMULATION_COMPLETE;
		}
	}

	m_stepState = retval;
	return retval;
}


template<class _type>
bool ScenarioTimeStep<_type>::CheckStops(HRESULT &condition) {
	if (!m_scenario->m_scenario->m_sc.anythingValid())
		return false;

	WTime time(m_time);
	if (m_scenario->m_scenario->m_sc.responseTime.GetTotalSeconds() > 0) {	// loop to look for the earliest response time that may have occurred
		ScenarioFire<_type>* sf = m_fires.LH_Head();
		bool responseValid = false;
		while (sf->LN_Succ()) {
			if (!responseValid)
				time = sf->Ignition()->m_ignitionTime + m_scenario->m_scenario->m_sc.responseTime;
			else {
				responseValid = true;
				WTime time2 = sf->Ignition()->m_ignitionTime + m_scenario->m_scenario->m_sc.responseTime;
				if (time2 < time)
					time = time2;
			}
			sf = sf->LN_Succ();
		}
	}
	if (m_time < time)
		return false;		// response time hasn't expired yet

	// first we need to go off and get other stat's we may want to grab
	// precip from primary wx stream
	// Fi thresholds, etc.
	if (m_scenario->m_scenario->m_sc.precip) {
		XY_Point pt(0.0, 0.0);
		NumericVariant nv;
		grid::AttributeValue av;
		if (SUCCEEDED(m_scenario->m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_scenario->m_layerThread, pt, m_time, m_scenario->m_scenario->m_sc.PrecipDuration, CWFGM_WEATHER_OPTION_CUMULATIVE_RAIN, 0, &nv, &av, nullptr))) {
			double precip = std::get<double>(nv);
			m_stopConditions.precip = (precip <= m_scenario->m_scenario->m_sc.PrecipThreshold);
		}
	}

	if (m_scenario->m_scenario->m_sc.fi90) {
		WTime t(m_time);
		double stats;
		m_scenario->GetStats((std::uint32_t)-1, &t, CWFGM_FIRE_STAT_FI, false, 0.0, m_scenario->m_scenario->m_sc.fi90PercentThreshold, &stats);
		m_stopConditions.fi90 = (stats <= 90.0);
	}
	if (m_scenario->m_scenario->m_sc.fi95) {
		WTime t(m_time);
		double stats;
		m_scenario->GetStats((std::uint32_t)-1, &t, CWFGM_FIRE_STAT_FI, false, 0.0, m_scenario->m_scenario->m_sc.fi95PercentThreshold, &stats);
		m_stopConditions.fi95 = (stats <= 95.0);
	}
	if (m_scenario->m_scenario->m_sc.fi100) {
		WTime t(m_time);
		double stats;
		m_scenario->GetStats((std::uint32_t)-1, &t, CWFGM_FIRE_STAT_FI, false, 0.0, m_scenario->m_scenario->m_sc.fi100PercentThreshold, &stats);
		m_stopConditions.fi100 = (stats <= 99.9);
	}
	if (m_scenario->m_scenario->m_sc.area) {
		PolymorphicAttribute stats;
		WTime t(m_time);
		double dstats;
		if (SUCCEEDED(RetrieveStat(CWFGM_FIRE_STAT_AREA, &dstats))) {
			m_scenario->fromInternal2D(dstats);
			dstats = UnitConvert::convertUnit(dstats, STORAGE_FORMAT_HECTARE, STORAGE_FORMAT_M2);
			m_stopConditions.area = (dstats < m_scenario->m_scenario->m_sc.areaThreshold);
		}
	}
	if (m_scenario->m_scenario->m_sc.burnDistance) {
		PolymorphicAttribute stats;
		WTime t(m_time);
		double dstats;
		if (SUCCEEDED(RetrieveStat(CWFGM_FIRE_STAT_MAXIMUM_BURN_DISTANCE, &dstats))) {
			m_scenario->fromInternal1D(dstats);
			m_stopConditions.burnDistance = (dstats < m_scenario->m_scenario->m_sc.burnDistanceThreshold);
		}
	}

	// now that we have this done, we have to look from present into past to see if the duration has been met
	XY_PointTempl<_type> centroid;
	this->Centroid(&centroid);
	if (m_scenario->CanBurn(m_time, centroid)) {
		if ((m_scenario->m_scenario->m_sc.precip) && (!m_stopConditions.precip)) {
			condition = SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_PRECIP;
			return true;
		}
		if ((m_scenario->m_scenario->m_sc.RH) && (!m_stopConditions.RH)) {
			WTime st = m_time - m_scenario->m_scenario->m_sc.RHDuration;
			ScenarioTimeStep* sts = LN_Pred();
			bool found = false, time_expired = false;
			while (sts->LN_Pred()) {
				if (sts->m_time < st) {									// in this condition, we've gone back far enough and found no state saying we can continue to burn
					time_expired = true;
					break;
				}
				if ((sts->m_stopConditions.RH) || (!sts->canBurn())) {	// if we can continue to burn...or we are in a non-burning-period, then abort and check the next condition
					found = true;
					break;
				}
				sts = sts->LN_Pred();
			}
			if ((!found) && (time_expired)) {	// if not found, then what that means is we've looped
				condition = SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_RH;
				return true;
			}
		}

		if ((m_scenario->m_scenario->m_sc.fi90) && (!m_stopConditions.fi90)) {
			WTime st = m_time - m_scenario->m_scenario->m_sc.fi90PercentDuration;
			ScenarioTimeStep* sts = LN_Pred();
			bool found = false, time_expired = false;
			while (sts->LN_Pred()) {
				if (sts->m_time < st) {									// in this condition, we've gone back far enough and found no state saying we can continue to burn
					time_expired = true;
					break;
				}
				if ((sts->m_stopConditions.fi90) || (!sts->canBurn())) {
					found = true;
					break;
				}
				sts = sts->LN_Pred();
			}
			if ((!found) && (time_expired)) {	// if not found, then what that means is we've looped
				condition = SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI90;
				return true;
			}
		}

		if ((m_scenario->m_scenario->m_sc.fi95) && (!m_stopConditions.fi95)) {
			WTime st = m_time - m_scenario->m_scenario->m_sc.fi95PercentDuration;
			ScenarioTimeStep* sts = LN_Pred();
			bool found = false, time_expired = false;
			while (sts->LN_Pred()) {
				if (sts->m_time < st) {									// in this condition, we've gone back far enough and found no state saying we can continue to burn
					time_expired = true;
					break;
				}
				if ((sts->m_stopConditions.fi95) || (!sts->canBurn())) {
					found = true;
					break;
				}
				sts = sts->LN_Pred();
			}
			if ((!found) && (time_expired)) {	// if not found, then what that means is we've looped
				condition = SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI95;
				return true;
			}
		}

		if ((m_scenario->m_scenario->m_sc.fi100) && (!m_stopConditions.fi100)) {
			WTime st = m_time - m_scenario->m_scenario->m_sc.fi100PercentDuration;
			ScenarioTimeStep* sts = LN_Pred();
			bool found = false, time_expired = false;
			while (sts->LN_Pred()) {
				if (sts->m_time < st) {									// in this condition, we've gone back far enough and found no state saying we can continue to burn
					time_expired = true;
					break;
				}
				if ((sts->m_stopConditions.fi100) || (!sts->canBurn())) {
					found = true;
					break;
				}
				sts = sts->LN_Pred();
			}
			if ((!found) && (time_expired)) {	// if not found, then what that means is we've looped
				condition = SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI100;
				return true;
			}
		}

		if ((m_scenario->m_scenario->m_sc.area) && (!m_stopConditions.area)) {
			condition = SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_AREA;
			return true;
		}

		if ((m_scenario->m_scenario->m_sc.burnDistance) && (!m_stopConditions.burnDistance)) {
			condition = SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_BURNDISTANCE;
			return true;
		}
	}

	return false;
}


template<class _type>
ScenarioTimeStep<_type>* Scenario<_type>::Purge() {
	ScenarioTimeStep<_type> *sts = m_timeSteps.LH_Tail();
	ScenarioTimeStep<_type> *p = sts->LN_Pred(), *pp;
	while (p->LN_Pred()) {
		p->m_lock.Lock_Write();

		if (p->m_displayable) {
			pp = p->LN_Pred();
			p->m_lock.Unlock();
			p = pp;
		} else {
			bool can_delete = true;
			{
				ActiveFire<_type> *af = m_activeFires.LH_Head();
				while (af->LN_Succ()) {
					ScenarioFire<_type> *sf = p->m_fires.LH_Head();
					while (sf->LN_Succ()) {
						if (sf == af->LN_Ptr()) {
							can_delete = false;
							break;
						}
						sf = sf->LN_Succ();
					}
					af = af->LN_Succ();
				}
			}
			if (can_delete) {
				ScenarioFire<_type> *sf = sts->m_fires.LH_Head();
				while (sf->LN_Succ()) {
					FireFront<_type> *ff = sf->LH_Head();
					while (ff->LN_Succ()) {
						FirePoint<_type> *fp = ff->LH_Head();
						while (fp->LN_Succ()) {
							if (fp->m_succPoint)		fp->m_succPoint->m_prevPoint = fp->m_prevPoint;
							if (fp->m_prevPoint)		fp->m_prevPoint->m_succPoint = fp->m_succPoint;
							fp = fp->LN_Succ();
						}
						ff = ff->LN_Succ();
					}
					if (sf->LN_CalcSucc())	sf->LN_CalcSucc()->setCalcPred(sf->LN_CalcPred());
					if (sf->LN_CalcPred())	sf->LN_CalcPred()->setCalcSucc(sf->LN_CalcSucc());
					sf->setCalcPred(nullptr);
					sf->setCalcSucc(nullptr);

					sf = sf->LN_Succ();
				}
				pp = p->LN_Pred();
				m_timeSteps.Remove(p);
				p->m_lock.Unlock();
				delete p;
				p = pp;
			} else {
				pp = p->LN_Pred();
				p->m_lock.Unlock();
				p = pp;
			}
		}
	}
	return nullptr;
}


template<class _type>
WTime Scenario<_type>::CurrentTime() const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	if (!m_timeSteps.GetCount())
		return m_scenario->m_startTime;

	ScenarioTimeStep<_type> *sts = m_timeSteps.LH_Tail();
	while (sts->LN_Pred()) {
		if ((sts->m_lock.CurrentState() >= 0) && (sts->m_displayable))
			return sts->m_time;
		sts = sts->LN_Pred();
	}
	return m_scenario->m_startTime;
}


template<class _type>
HRESULT Scenario<_type>::StepBack() {
	CRWThreadSemaphoreEngage _semaphore_engageS(m_stepLock, SEM_TRUE);
	CRWThreadSemaphoreEngage _semaphore_engage(m_llLock, SEM_TRUE);

	std::uint16_t remove_display = 0;
	ScenarioTimeStep<_type> *sts = m_timeSteps.LH_Tail();
	while (sts->LN_Pred()) {
		if ((remove_display) || (sts->m_displayable)) {
			if ((remove_display) && (sts->m_displayable))	// remove only one displayable time step when stepping backwards
				break;
			m_timeSteps.RemTail();
			delete sts;
			remove_display++;
			sts = m_timeSteps.LH_Tail();
		} else {
			weak_assert(0);					// why would this ever happen
			sts = sts->LN_Pred();
		}
	}
	if (sts->LN_Pred())					// if the list isn't empty...
		sts->RestoreActiveFires();		// then reset to that state
	else {								// otherwise, we're at the start, so there won't be anything on the m_activeFires list
		ActiveFire<_type>* af;
		while (af = m_activeFires.RemHead()) {
			delete af;
		}
	}
	return S_OK;
}


template<class _type>
HRESULT Scenario<_type>::GetStep(WTime *time, ScenarioTimeStep<_type> **sts, const bool only_displayable) const {
	if (m_timeSteps.IsEmpty()) {
		*sts = nullptr;
		return (ERROR_NO_DATA | ERROR_SEVERITY_WARNING);	// nothing initialized yet
	}
	*sts = m_timeSteps.LH_Tail();
	while ((*sts)->LN_Pred()) {
		if ((*sts)->m_lock.CurrentState() >= 0)
			if ((!only_displayable) || ((*sts)->m_displayable)) {
				WTime ltime(*time, m_scenario->m_timeManager);
				if ((*sts)->m_time <= ltime) {
					time->SetTime((*sts)->m_time);		// reset the time to the appropriate thing
					return S_OK;
				}
			}
		*sts = (*sts)->LN_Pred();
	}
	*sts = nullptr;
	return SUCCESS_FIRE_NOT_STARTED;				// either no fires or the request predates the start time of the simulation
}


template<class _type>
ScenarioTimeStep<_type> *Scenario<_type>::GetPreviousStep(ScenarioTimeStep<_type> *sts, bool only_displayable, const FireFront<_type> *ff) const {
	ScenarioFire<_type>* sf;
	if (ff == nullptr) {							// this is telling us we don't care which previous timestep, so long as it's a timestep that's prior to sts, so I'll pick the most recent
		sts = sts->LN_Pred();
		while (sts->LN_Pred()) {
			if ((!only_displayable) || (sts->m_displayable))
				return sts;
			sts = sts->LN_Pred();
		}
		return nullptr;
	}
	else {

		ScenarioFire<_type>* sf = const_cast<ScenarioFire<_type>*>(ff->Fire());

#ifdef _DEBUG
		bool found = false;
		ScenarioFire<_type>* f = sts->m_fires.LH_Head();
		while (f->LN_Succ()) {
			if (f == sf) {
				found = true;
				break;
			}
			f = f->LN_Succ();
		}
		weak_assert(found);
#endif

		sf = sf->LN_CalcPred();
		while (sf) {
			if ((!only_displayable) || (sf->TimeStep()->m_displayable))
				return const_cast<ScenarioTimeStep<_type>*>(sf->TimeStep());
			sf = sf->LN_CalcPred();
		}
		return nullptr;
	}
}


template<class _type>
ScenarioTimeStep<_type>* Scenario<_type>::GetPreviousDisplayStep(ScenarioTimeStep<_type>* sts, FireFront<_type>* closest_ff, ScenarioTimeStep<_type>* sts_prev) const {
	ScenarioTimeStep<_type>* sss = sts->LN_Pred();
	while (sss->LN_Pred()) {
		if (sss == sts_prev)
			return nullptr;							// didn't find anything
		ScenarioFire<_type>* sf = sss->m_fires.LH_Head();
		while (sf->LN_Succ()) {
			if (sf->LN_CalcPred() == closest_ff->Fire()->LN_CalcPred()) {
				weak_assert(sss->m_time > sts_prev->m_time);
				weak_assert(sss->m_time < sts->m_time);
				return sss;
			}
			sf = sf->LN_Succ();
		}
		sss = sss->LN_Pred();
	}
	weak_assert(0);
	return nullptr;
}


template<class _type>
HRESULT Scenario<_type>::GetBurningBox(WTime *time, XYRectangleType &bbox) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	ScenarioTimeStep<_type> *sts;
	HRESULT hr = GetStep(time, &sts, true);

	if (!sts) {
		bbox.m_min.y = bbox.m_min.x = DBL_MAX;
		bbox.m_max.y = bbox.m_max.x = -DBL_MAX;
		return hr;
	}
	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);
	return (sts->BoundingBox(bbox)) ? S_OK : ERROR_SEVERITY_WARNING;
}


template<class _type>
HRESULT Scenario<_type>::PointBurned(const XYPointType &pt, WTime *time, bool *status) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	ScenarioTimeStep<_type> *sts;
	HRESULT hr = GetStep(time, &sts, true);

	if (!sts) {
		*status = false;
		return hr;
	}

	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);
	*status = sts->PointInArea(pt) ? true : false;
	return S_OK;
}


template<class _type>
HRESULT Scenario<_type>::GetNumSteps(std::uint32_t *size) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	std::uint32_t cnt = 0;
	ScenarioTimeStep<_type> *sts = m_timeSteps.LH_Head();
	while (sts->LN_Succ()) {
		if (sts->m_displayable)
			cnt++;
		sts = sts->LN_Succ();
	}
	*size = cnt;
	return S_OK;
}


template<class _type>
HRESULT Scenario<_type>::GetStepsArray(std::uint32_t *size, std::vector<WTime> *times) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	std::uint32_t cnt = 0;
	ScenarioTimeStep<_type> *sts = m_timeSteps.LH_Head();

	while ((sts->LN_Succ()) && (cnt < (*size))) {
		if (sts->m_displayable)
			(*times)[cnt++].SetTime(sts->m_time);
		sts = sts->LN_Succ();
	}
	*size = cnt;
	return S_OK;
}


template<class _type>
HRESULT Scenario<_type>::GetNumFires(std::uint32_t *count, WTime *time) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	ScenarioTimeStep<_type> *sts;
	HRESULT hr = GetStep(time, &sts, true);

	if (!sts) {
		*count = 0;
		return hr;
	}
	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);
	*count = sts->GetNumFireFronts();
	return S_OK;
}


template<class _type>
HRESULT Scenario<_type>::GetIgnition(std::uint32_t fire, WTime *time, boost::intrusive_ptr<CCWFGM_Ignition> *ignition) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);
	ScenarioTimeStep<_type> *sts;
	HRESULT hr = GetStep(time, &sts, true);

	if (!sts) {
		*ignition = nullptr;
		return hr;
	}
	CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE);
	const FireFront<_type> *ff = sts->GetFireFront(fire);
	if (!ff) {
		*ignition = nullptr;
		return ERROR_SCENARIO_FIRE_UNKNOWN;
	}
	*ignition = ff->Fire()->Ignition()->m_ignitionCOM;
	return S_OK;
}


template<class _type>
bool ScenarioFireExport<_type>::createExportFields(const TCHAR *driver_name, OGRLayerH layer) {
	if (!XY_PolyLL_Set<FireFront<_type>, _type>::createExportFields(driver_name, layer))
		return false;
	
	const ScenarioExportRules* rules = m_rules;

	if (!rules->GetAppendOperation())
		if (!XY_PolyLL_Set<FireFront<_type>, _type>::NumPolys())
			return false;

	std::uint32_t counter = 0;
	ExportRule *r = rules->GetNextAttributeName(counter);
	while (r) {
		if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY) {
			std::uint32_t len = rules->GetTextAttributeMaxLength(r->name.c_str());
			OGRFieldDefnH field = OGR_Fld_Create(r->name.c_str(), OFTString);
			OGR_Fld_SetWidth(field, ((len + 1) & (~31)) + 32);
			if (OGR_L_CreateField(layer, field, TRUE) != OGRERR_NONE)
				return false;
			OGR_Fld_Destroy(field);
		} else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_DOUBLEPROPERTY) {
			OGRFieldDefnH field = OGR_Fld_Create(r->name.c_str(), OFTReal);
			if (OGR_L_CreateField(layer, field, TRUE) != OGRERR_NONE)
				return false;
			OGR_Fld_Destroy(field);
		} else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_INT32PROPERTY) {
			OGRFieldDefnH field = OGR_Fld_Create(r->name.c_str(), OFTInteger);
			if (OGR_L_CreateField(layer, field, TRUE) != OGRERR_NONE)
				return false;
			OGR_Fld_Destroy(field);
		} else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_INT64PROPERTY) {
			OGRFieldDefnH field = OGR_Fld_Create(r->name.c_str(), OFTInteger64);
			if (OGR_L_CreateField(layer, field, TRUE) != OGRERR_NONE)
				return false;
			OGR_Fld_Destroy(field);
		} else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_STATPROPERTY) {
			OGRFieldType type;
			std::uint64_t op;
			int width = -1;
			VariantToUInt64_(r->value, &op);
			std::uint16_t stat = (std::uint16_t)(op & 0xffff);
			if (stat == CWFGM_FIRE_STAT_DATETIME)
				type = OFTDateTime;
			else if (stat == CWFGM_FIRE_STAT_DATE)
				type = OFTString;
			else if ((stat == CWFGM_FIRE_STAT_TIME) || (stat == CWFGM_FIRE_STAT_ASSET_FIRST_ARRVIAL_TIME)) {
				type = OFTString;
				width = 10;
			} else if ((stat == CWFGM_FIRE_STAT_ASSET_ARRIVAL_COUNT) ||
				(stat == CWFGM_FIRE_STAT_ASSET_FIRST_ARRVIAL_SECS) ||
				(stat == CWFGM_FIRE_STAT_FINAL_PERIMETER) ||
				(stat == CWFGM_FIRE_STAT_SIMULATION_STATUS)) {
				type = OFTInteger;
			} else
				type = OFTReal;
			OGRFieldDefnH field = OGR_Fld_Create(r->name.c_str(), type);
			if (width != -1)
				OGR_Fld_SetWidth(field, (width & (~31)) + 32);
			OGRErr err = OGR_L_CreateField(layer, field, TRUE);
			if (err != OGRERR_NONE) {

				OGR_Fld_Destroy(field);
				return false;
			}
			OGR_Fld_Destroy(field);
		}
		r = rules->GetNextAttributeName(counter);
	}

	return true;
}


template<class _type>
bool ScenarioFireExport<_type>::exportPolygon(XYPolyLLType *externalPoly) const {
	if (externalPoly->m_publicFlags & 0x8000)
		return false;		// already exported
	externalPoly->m_publicFlags |= 0x8000;
	return true;
}


template<class _type>
bool ScenarioFireExport<_type>::associatePolygon(const XYPolyLLType *externalPoly, const XYPolyLLType *internalPoly) const {
	FireFrontExport<_type>	*externalFF = (FireFrontExport<_type>*)externalPoly,
							*internalFF = (FireFrontExport<_type>*)internalPoly;
	if ((!externalFF->m_origScenarioFire) || (!internalFF->m_origScenarioFire)) {

		return XY_PolyLL_Set<FireFront<_type>, _type>::associatePolygon(externalPoly, internalPoly);
	}
	if (externalFF->m_origScenarioFire == internalFF->m_origScenarioFire) {
		if ((externalPoly->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERPRET_POLYGON) &&
			(internalPoly->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERPRET_POLYGON)) {
			XYPointType pt = *internalPoly->LH_Head();
			return externalPoly->PointInArea(pt) ? true : false;
		}
		return true;
	}
	return false;
}


template<class _type>
bool ScenarioFireExport<_type>::setExportFields(const TCHAR *driver_name, OGRFeatureH feature, XYPolyLLType *poly) {
	if (!XY_PolyLL_Set<FireFront<_type>, _type>::setExportFields(driver_name, feature, poly))
		return false;

	bool is_kml;
	if ((_tcscmp(driver_name, _T("LIBKML")) == 0) || (_tcscmp(driver_name, _T("KML")) == 0))
		is_kml = true;
	else
		is_kml = false;

	FireFrontExport<_type> *ff = (FireFrontExport<_type>*)poly;
	const ScenarioExportRules *rules = m_rules;

	ExportRule *r = rules->m_rules.LH_Head();
	while (r->LN_Succ()) {
		if ((!r->ignition)						// this rule applies to all ignitions
		    || ((ff->m_origScenarioFire) && (ff->m_origScenarioFire->Ignition()) && (ff->m_origScenarioFire->Ignition()->m_ignitionCOM == r->ignition))) {
			if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY) {
				int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
				std::string str;
				try {
					str = std::get<std::string>(r->value);
				}
				catch (std::bad_variant_access &) {
					weak_assert(0);
				}
				OGR_F_SetFieldString(feature, index, str.c_str());
			}
			else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_DOUBLEPROPERTY) {
				double val;
				VariantToDouble_(r->value, &val);
				int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
				OGR_F_SetFieldDouble(feature, index, val);
			}
			else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_INT32PROPERTY) {
				std::int32_t val;
				VariantToInt32_(r->value, &val);
				int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
				OGR_F_SetFieldInteger(feature, index, val);
			}
			else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_INT64PROPERTY) {
				std::int64_t val;
				VariantToInt64_(r->value, &val);
				int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
				OGR_F_SetFieldInteger64(feature, index, val);
			} else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_STATPROPERTY) {
				std::uint64_t op;
				VariantToUInt64_(r->value, &op);
				std::uint16_t stat = (std::uint16_t)(op & 0xffff);
				std::uint32_t units = (std::uint32_t)((op >> 32) & 0xffffffff);
				if ((stat == CWFGM_FIRE_STAT_DATETIME)) {
					WTime tt(ff->m_time);
					int year, month, day, hour, min, second;
					year = tt.GetYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
					month = tt.GetMonth(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
					day = tt.GetDay(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
					hour = tt.GetHour(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
					min = tt.GetMinute(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
					second = tt.GetSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
					int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
					OGR_F_SetFieldDateTime(feature, index, year, month, day, hour, min, second, 1);
				} else if (stat == CWFGM_FIRE_STAT_DATE) {
					WTime tt(ff->m_time);
					std::string time = tt.ToString(WTIME_FORMAT_DATE | WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST | WTIME_FORMAT_STRING_YYYYhMMhDD);
					int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
					OGR_F_SetFieldString(feature, index, time.c_str());
				} else if (stat == CWFGM_FIRE_STAT_TIME) {
					WTime tt(ff->m_time);
					std::string time = tt.ToString(WTIME_FORMAT_TIME | WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST | WTIME_FORMAT_EXCLUDE_SECONDS);
					int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
					OGR_F_SetFieldString(feature, index, time.c_str());
				} else if (stat == CWFGM_FIRE_STAT_ASSET_FIRST_ARRVIAL_TIME) {
					int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
					if (ff->m_assetCount > 0) {
						std::string time = ff->m_assetTime.ToString(WTIME_FORMAT_STRING_ISO8601);
						OGR_F_SetFieldString(feature, index, time.c_str());
					}
					else
						OGR_F_SetFieldNull(feature, index);
				} else if (stat == CWFGM_FIRE_STAT_ASSET_FIRST_ARRVIAL_SECS) {
					int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
					if (ff->m_assetCount > 0) {
						std::uint32_t time = ff->m_assetTime.GetTotalSeconds();
						OGR_F_SetFieldInteger(feature, index, time);
					} else
						OGR_F_SetFieldNull(feature, index);
				} else if (stat == CWFGM_FIRE_STAT_FINAL_PERIMETER) {
					int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
					int last = 0;
					if (ff->m_origScenarioFire) {
						if (!ff->m_origScenarioFire->TimeStep()->LN_Succ()->LN_Succ())
							last = 1;
					}
					OGR_F_SetFieldInteger(feature, index, last);
				} else if (stat == CWFGM_FIRE_STAT_SIMULATION_STATUS) {
					int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
					std::int32_t status = SUCCESS_SCENARIO_SIMULATION_RUNNING;
					if (ff->m_origScenarioFire) {
						if (!ff->m_origScenarioFire->TimeStep()->LN_Succ()->LN_Succ()) {
							if (ff->m_origScenarioFire->TimeStep()->m_scenario->m_stepState != S_OK) {	// if S_OK, then running, otherwise it's stopped, now I need to see if we're on the last timestep or nog
								status = ff->m_origScenarioFire->TimeStep()->m_scenario->m_stepState;
							}
						}
					}
					OGR_F_SetFieldInteger(feature, index, status);
				} else if (stat == CWFGM_FIRE_STAT_ASSET_ARRIVAL_COUNT) {
					int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
					OGR_F_SetFieldInteger(feature, index, ff->m_assetCount);
				} else {
					double dstats;
					XYPointType loc;
					Scenario<_type>* orig_s;
					bool can_output = true;
					switch (stat) {
						case CWFGM_FIRE_STAT_IGNITION_LATITUDE:
						case CWFGM_FIRE_STAT_IGNITION_LONGITUDE:
							if (ff->m_origScenarioFire)
							{
    							orig_s = ff->m_origScenarioFire->TimeStep()->m_scenario;
    							if (ff->m_origScenarioFire->Ignition()->getPoint(orig_s->m_scenario->m_dx,
    								orig_s->m_scenario->m_dy,
								    loc)) {
									XY_Point _loc(loc);
    								ff->m_origScenarioFire->TimeStep()->m_scenario->m_coordinateConverter.SourceToLatlon(1, &_loc.x, &_loc.y, nullptr);
    								if (stat == CWFGM_FIRE_STAT_IGNITION_LONGITUDE)
    									dstats = _loc.x;
    								else
    									dstats = _loc.y;
    							}
								else
									dstats = 0.0;
							}
							else
								dstats = 0.0;
							break;

						case CWFGM_FIRE_STAT_MAXIMUM_BURN_DISTANCE:
						case CWFGM_FIRE_STAT_ACTIVE_PERIMETER:
						case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER:			// return in meters
						case CWFGM_FIRE_STAT_TOTAL_PERIMETER:		
												if ((m_flags & SCENARIO_EXPORT_COMBINE_SET) &&
												    (!(m_flags & (SCENARIO_EXPORT_SUBSET_EXTERIOR | SCENARIO_EXPORT_SUBSET_ACTIVE)))) {
													switch (stat) {
														case CWFGM_FIRE_STAT_ACTIVE_PERIMETER:		dstats = ff->m_origActivePerimeter; break;
														case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER:	dstats = ff->m_origExteriorPerimeter; break;
														case CWFGM_FIRE_STAT_TOTAL_PERIMETER:		dstats = ff->m_origPerimeter; break;
														case CWFGM_FIRE_STAT_MAXIMUM_BURN_DISTANCE:	dstats = ff->m_origDistance; break;
													}
												} else if (stat == CWFGM_FIRE_STAT_MAXIMUM_BURN_DISTANCE) {
													if (ff->m_origScenarioFire)
													{
      													orig_s = ff->m_origScenarioFire->TimeStep()->m_scenario;
      													if (ff->m_origScenarioFire->Ignition()->getPoint(orig_s->m_scenario->m_dx,
      														orig_s->m_scenario->m_dy,
      														loc)) {
														  _type qstats;
														  ff->FurthestPoint(loc, nullptr, &qstats);
														  dstats = (double)qstats;
													   }
														else
															dstats = 0.0;
													}
													else
														dstats = 0.0;
												} else {
													if (ff->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERPRET_POLYLINE) {
														weak_assert(ff->IsPolyline());
														dstats = ff->Length();
													} else {
														ff->RetrieveStat(stat, &dstats);
														if (!is_kml) {
															XYPolyLLType *p = poly->LN_SuccWrap();
															while (p != poly) {
																if (((p->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERPRET_POLYGON)
																    && (p->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERIOR_SPECIFIED))
																    && (!p->IsEmpty()))
																	if (associatePolygon(poly, p)) {
																		FireFrontExport<_type> *ff1 = (FireFrontExport<_type>*)p;
																		double dstats1;
																		if (SUCCEEDED(ff1->RetrieveStat(stat, &dstats1))) {

																			dstats += dstats1;
																		}
																	}
																p = p->LN_SuccWrap();
															}
														}
													}
												}
												dstats = UnitConvert::convertUnit(dstats, units, STORAGE_FORMAT_M);
												dstats = ROUND_DECIMAL(dstats, 2);
												break;
						case CWFGM_FIRE_STAT_AREA:	
												if ((m_flags & SCENARIO_EXPORT_COMBINE_SET) &&
												    (!(m_flags & (SCENARIO_EXPORT_SUBSET_EXTERIOR | SCENARIO_EXPORT_SUBSET_ACTIVE))))
													dstats = ff->m_origArea;
												else {
													if (ff->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERPRET_POLYLINE) {
														can_output = false;
														dstats = 0.0;
													} else {
														dstats = ff->Area();

														if ((dstats > 0.0) && (ff->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERIOR_SPECIFIED))
															dstats = -dstats;
														else if ((dstats < 0.0) && (!(ff->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERIOR_SPECIFIED)))
															dstats = -dstats;
														if (!is_kml) {
															XYPolyLLType *p = poly->LN_SuccWrap();
															while (p != poly) {
																if (((p->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERPRET_POLYGON)
																    && (p->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERIOR_SPECIFIED))
																    && (!p->IsEmpty()))
																	if (associatePolygon(poly, p)) {
																		FireFrontExport<_type> *ff1 = (FireFrontExport<_type>*)p;
																		double dstats1;
																		dstats1 = ff1->Area();
																		if (dstats1 > 0.0)
																			dstats1 = -dstats1;
																			dstats += dstats1;
																	}
																p = p->LN_SuccWrap();
															}
														}
													}
												}
												dstats = UnitConvert::convertUnit(dstats, units, STORAGE_FORMAT_M2);
												dstats = ROUND_DECIMAL(dstats, 2);
												break;
					}
					if (can_output) {
						int index = OGR_F_GetFieldIndex(feature, r->name.c_str());
						OGR_F_SetFieldDouble(feature, index, dstats);
					}
				}
			}
		}
		r = r->LN_Succ();
	}

	return true;
}


template<class _type>
class vector_append {
public:
	vector_append(const ScenarioExportRules& r) : combined_set(nullptr, nullptr, nullptr), rules(r) { hr = 0; combined_set.m_rules = &rules; context = nullptr; }
	ScenarioFireExport<_type> combined_set;
	ScenarioExportRules rules;
	OGRSpatialReferenceH oSourceSRS, oTargetSRS;
	void* context;
	HRESULT hr;
};


template<class _type>
HRESULT Scenario<_type>::Export(const CCWFGM_Ignition *ignition, WTime *start_time, WTime *end_time, std::uint16_t flags,
	const TCHAR *driver_name, const TCHAR *csProjection, const TCHAR *file_path,
    const ScenarioExportRules &rules, ScenarioTimeStep<_type> *_sts) const {
	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore *)&m_llLock, SEM_FALSE);

	HRESULT hr;
	PolymorphicAttribute var;
	if (FAILED(hr = m_scenario->m_gridEngine->GetAttribute(m_scenario->m_layerThread, CWFGM_GRID_ATTRIBUTE_SPATIALREFERENCE, &var))) return hr;
	std::string projection;
	try { projection = std::get<std::string>(var); } catch (std::bad_variant_access &) { weak_assert(0); return ERROR_PROJECTION_UNKNOWN; }; /*POLYMORPHIC*/

	CSemaphoreEngage lock(GDALClient::GDALClient::getGDALMutex(), true);

	OGRSpatialReferenceH oSourceSRS = CCoordinateConverter::CreateSpatialReferenceFromWkt(projection.c_str());
	OGRSpatialReferenceH oTargetSRS = CCoordinateConverter::CreateSpatialReferenceFromStr(csProjection);

	ScenarioTimeStep<_type> *sts;
	if (!_sts) {
		if ((*start_time) == (*end_time)) {
			HRESULT hr = GetStep(start_time, &sts, true);
			if (((FAILED(hr)) || (!sts)) && (!rules.GetAppendOperation())) {
				if (oSourceSRS)
					OSRDestroySpatialReference(oSourceSRS);
				if (oTargetSRS)
					OSRDestroySpatialReference(oTargetSRS);
				return hr;
			}
			if (sts) {
				*start_time = sts->m_time;
				*end_time = sts->m_time;
			}
		}
	} else {
		*start_time = _sts->m_time;
		*end_time = _sts->m_time;
	}

	WTime requestedEndTime(*end_time);
	ScenarioFireExport<_type> full_set(nullptr, nullptr, nullptr);

	if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
		full_set.SetCacheScale(resolution());
	full_set.m_rules = &rules;
	bool first = true;
	for (sts = m_timeSteps.LH_Head(); sts->LN_Succ(); sts = sts->LN_Succ()) {
		if (_sts) {
			if (sts != _sts)
				continue;
		} else {
			if (!sts->m_displayable)
				continue;
			if (sts->m_time < (*start_time))
				continue;
			if (sts->m_time > requestedEndTime)
				continue;
		}
		if (first) {
			*start_time = sts->m_time;
			first = false;
		}
		*end_time = sts->m_time;

		{
			bool try_success;
			CRWThreadSemaphoreEngage _semaphore_engage2(sts->m_lock, SEM_FALSE, (_sts) ? &try_success : nullptr);

			IgnitionNode<_type> *node;
			if (!ignition)
				node = nullptr;
			else {
				node = m_scenario->m_impl->m_ignitionList.LH_Head();
				while (node->LN_Succ()) {
					if (node->m_ignitionCOM == ignition)
						break;
					node = node->LN_Succ();
				}
				if (!node->LN_Succ())
					node = nullptr;
			}

			ScenarioFireExport<_type> set(sts, node, nullptr);

			set.m_rules = &rules;
			ScenarioFire<_type> *sf = sts->m_fires.LH_Head();
			while (sf->LN_Succ()) {
				bool to_add = true;
				if ((ignition) && (ignition != sf->Ignition()->m_ignitionCOM))
					to_add = false;
				if (to_add) {
					FireFront<_type> *ff = sf->LH_Head();
					while (ff->LN_Succ()) {
						if (!((flags & SCENARIO_EXPORT_SUBSET_EXTERIOR) && (ff->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERIOR_SPECIFIED))) {
							FireFrontExport<_type> *copy = new FireFrontExport<_type>(nullptr, *ff);
							copy->m_time = sts->m_time;
							if (copy->m_assetCount = sts->m_assetCount) {
								WTime t((std::uint64_t)0, m_scenario->m_timeManager);
								IgnitionNode<_type>* ig = m_scenario->m_impl->m_ignitionList.LH_Head();
								while (ig->LN_Succ()) {
									if (!t.GetTotalMicroSeconds())
										t = ig->m_ignitionTime;
									else if (t > ig->m_ignitionTime)
										t = ig->m_ignitionTime;
									ig = ig->LN_Succ();
								}
								if (t < m_scenario->m_startTime)
									t = m_scenario->m_startTime;
								copy->m_assetTime = copy->m_time - t;
							}
							if (!(flags & SCENARIO_EXPORT_COMBINE_SET))
								copy->m_origScenarioFire = sf;
							FirePoint<_type> *fp = ff->LH_Head(), *new_fp = copy->LH_Head();
							while (fp->LN_Succ()) {
								new_fp->m_ellipse_ros = fp->m_ellipse_ros;
								new_fp->m_fbp_raz = fp->m_fbp_raz;
								new_fp->m_fbp_rsi = fp->m_fbp_rsi;
								new_fp->m_fbp_ros = fp->m_fbp_ros;
								new_fp->m_fbp_bros = fp->m_fbp_bros;
								new_fp->m_fbp_fros = fp->m_fbp_fros;
								new_fp->m_vector_ros = fp->m_vector_ros;
								new_fp->m_vector_cfb = fp->m_vector_cfb;
								new_fp->m_vector_cfc = fp->m_vector_cfc;
								new_fp->m_vector_sfc = fp->m_vector_sfc;
								new_fp->m_vector_tfc = fp->m_vector_tfc;
								new_fp->m_vector_fi = fp->m_vector_fi;
								new_fp->m_fbp_fi = fp->m_fbp_fi;
								new_fp->m_fbp_cfb = fp->m_fbp_cfb;
								new_fp->m_fbp_ros_ratio = fp->m_fbp_ros_ratio;
								fp = fp->LN_Succ();
								new_fp = new_fp->LN_Succ();
							}
							set.AddPoly(copy);
						}
						ff = ff->LN_Succ();
					}
				}
				sf = sf->LN_Succ();
			}

			if (!(m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
				set.SetCacheScale(resolution());
	
			full_set.m_flags = flags;

			double	m_origArea, m_origPerimeter, m_origExteriorPerimeter, m_origActivePerimeter, m_origDistance;

			m_origArea = set.Area();
			m_origPerimeter = set.TotalLength();
			set.RetrieveStat(CWFGM_FIRE_STAT_EXTERIOR_PERIMETER, &m_origExteriorPerimeter);
			set.RetrieveStat(CWFGM_FIRE_STAT_ACTIVE_PERIMETER, &m_origActivePerimeter);
			set.RetrieveStat(CWFGM_FIRE_STAT_MAXIMUM_BURN_DISTANCE, &m_origDistance);

			fromInternal1D(m_origPerimeter);
			fromInternal1D(m_origExteriorPerimeter);
			fromInternal1D(m_origActivePerimeter);
			fromInternal2D(m_origArea);
			fromInternal1D(m_origDistance);

			if (flags & SCENARIO_EXPORT_COMBINE_SET) {
				set.Unwind(false, ScenarioCache<_type>::m_multithread ? true : false, nullptr, nullptr);
			}

			if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING))
				set.SetCacheScale(resolution());

			if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)) {
				set.ScaleXY(resolution());
			}
			if (m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN)) {
				XYPointType ll(start_ll());
				set.TranslateXY(ll);
			}

			if (flags & SCENARIO_EXPORT_SUBSET_ACTIVE) {
				ScenarioFireExport<_type> lineset(sts, node, nullptr);

				lineset.SetCacheScale(resolution());
				lineset.m_rules = &rules;
				FireFrontExport<_type> *ff = set.LH_Head();
				FireFrontExport<_type> *line = nullptr;
				while (ff->LN_Succ()) {
					FirePoint<_type> *fp = ff->LH_Tail();
					bool loop = true;
					while ((loop) || (fp->LN_Succ())) {
						if ((fp->m_status == FP_FLAG_NORMAL) || (fp->LN_SuccWrap()->m_status == FP_FLAG_NORMAL) || (fp->LN_PredWrap()->m_status == FP_FLAG_NORMAL)) {
							if (!line) {
								if (!((!loop) && (fp == ff->LH_Tail()))) {	// no use starting a line for one last point - that we had actually started on anyway
									line = new FireFrontExport<_type>();
										line->SetCacheScale(resolution());
									line->m_origScenarioFire = ff->m_origScenarioFire;
									line->m_publicFlags |= XY_PolyLL_BaseTempl<_type>::Flags::INTERPRET_POLYLINE;
								}
							}
							if (line) {
								FirePoint<_type> *new_fp = new FirePoint<_type>(*fp);
								new_fp->m_ellipse_ros = fp->m_ellipse_ros;
								new_fp->m_fbp_raz = fp->m_fbp_raz;
								new_fp->m_fbp_rsi = fp->m_fbp_rsi;
								new_fp->m_fbp_ros = fp->m_fbp_ros;
								new_fp->m_fbp_bros = fp->m_fbp_bros;
								new_fp->m_fbp_fros = fp->m_fbp_fros;
								new_fp->m_vector_ros = fp->m_vector_ros;
								new_fp->m_vector_cfb = fp->m_vector_cfb;
								new_fp->m_vector_cfc = fp->m_vector_cfc;
								new_fp->m_vector_sfc = fp->m_vector_sfc;
								new_fp->m_vector_tfc = fp->m_vector_tfc;
								new_fp->m_vector_fi = fp->m_vector_fi;
								new_fp->m_fbp_fi = fp->m_fbp_fi;
								new_fp->m_fbp_cfb = fp->m_fbp_cfb;
								new_fp->m_fbp_ros_ratio = fp->m_fbp_ros_ratio;
								line->AddTail(new_fp);
							}
						} else {
							if ((line) && (line->NumPoints() > 1)) {
								lineset.AddPoly(line);
								line = nullptr;
							} else {
								if ((line) && (fp != ff->LH_Head()))
									weak_assert(0);
								delete line;
								line = nullptr;
							}
						}
						if (loop) {
							loop = false;
							fp = fp->LN_SuccWrap();
						} else	fp = fp->LN_Succ();
					}
					if ((line) && (line->NumPoints() > 1)) {
						lineset.AddPoly(line);
						line = nullptr;
					} else {
						if (line)
							weak_assert(false);
						delete line;
						line = nullptr;
					}
					ff = ff->LN_Succ();
				}
				while (ff = (FireFrontExport<_type>*)lineset.RemHead()) {
					ff->m_time = sts->m_time;
					if (ff->m_assetCount = sts->m_assetCount) {
						WTime t((ULONGLONG)0, m_scenario->m_timeManager);
						IgnitionNode<_type>* ig = m_scenario->m_impl->m_ignitionList.LH_Head();
						while (ig->LN_Succ()) {
							if (!t.GetTotalMicroSeconds())
								t = ig->m_ignitionTime;
							else if (t > ig->m_ignitionTime)
								t = ig->m_ignitionTime;
							ig = ig->LN_Succ();
						}
						if (t < m_scenario->m_startTime)
							t = m_scenario->m_startTime;
						ff->m_assetTime = ff->m_time - t;
					}
					ff->m_origArea = m_origArea;
					ff->m_origActivePerimeter = m_origActivePerimeter;
					ff->m_origExteriorPerimeter = m_origExteriorPerimeter;
					ff->m_origPerimeter = m_origPerimeter;
					ff->m_origDistance = m_origDistance;
					full_set.AddPoly(ff);
				}
			} else {
				FireFrontExport<_type> *ff;
				while (ff = (FireFrontExport<_type>*)set.RemHead()) {
					ff->m_publicFlags |= XY_PolyLL_BaseTempl<_type>::Flags::INTERPRET_POLYGON;
					ff->m_time = sts->m_time;
					ff->m_origArea = m_origArea;
					ff->m_origActivePerimeter = m_origActivePerimeter;
					ff->m_origExteriorPerimeter = m_origExteriorPerimeter;
					ff->m_origPerimeter = m_origPerimeter;
					ff->m_origDistance = m_origDistance;
					full_set.AddPoly(ff);
				}
			}
		}
	}

	ULONGLONG byref, byref2;
	vector_append<_type>** _byref;

	if (byref = rules.GetAppendOperation()) {
		_byref = (vector_append<_type>**)byref;
		vector_append<_type>* va = nullptr;
		if (!(*_byref)) {
			va = new vector_append<_type>(rules);
			*_byref = va;
			va->oSourceSRS = oSourceSRS;
			va->oTargetSRS = oTargetSRS;
			hr = va->hr = va->combined_set.ExportPoly_Open(driver_name, file_path, &va->context, va->oSourceSRS, va->oTargetSRS);
		}
		else
			va = (*_byref);

		if (SUCCEEDED(va->hr))
			hr = va->hr = full_set.ExportPoly_Append(&va->context);

		if (byref2 = rules.GetExportOperation()) {
			weak_assert(byref2 == byref);
			if (SUCCEEDED(va->hr))
				hr = va->hr = va->combined_set.ExportPoly_Complete(&va->context);

			if (va->oSourceSRS)
				OSRDestroySpatialReference(va->oSourceSRS);
			if (va->oTargetSRS)
				OSRDestroySpatialReference(va->oTargetSRS);

			*_byref = nullptr;
			delete va;
		}
	}
	else if (SUCCEEDED(hr)) {
		hr = full_set.ExportPoly(driver_name, file_path, oSourceSRS, oTargetSRS);

		if (oSourceSRS)
			OSRDestroySpatialReference(oSourceSRS);
		if (oTargetSRS)
			OSRDestroySpatialReference(oTargetSRS);
	}
	return hr;
}


template<class _type>
HRESULT Scenario<_type>::BuildCriticalPath(const AssetNode<_type>* node, const AssetGeometryNode<_type>* g, const std::uint16_t flags, CriticalPath* polyset, const ScenarioExportRules* rules) const {
	if (!g->m_arrived)
		return ERROR_SCENARIO_ASSET_NOT_ARRIVED;

	g->BuildCriticalPath(m_scenario->m_timeManager, *polyset, rules);

	CriticalPathPoint *poly = (CriticalPathPoint*)polyset->LH_Head();
	while (poly->LN_Succ()) {
		CriticalPathPoint* poly2 = poly->LN_Succ();
		while (poly2->LN_Succ()) {
			CriticalPathPointData* ptdata = (CriticalPathPointData*)poly->LH_Head();
			CriticalPathPointData* ptdata2 = (CriticalPathPointData*)poly2->LH_Head();
			if ((ptdata->LN_Succ()) && (ptdata2->LN_Succ()) && (ptdata->x == ptdata2->x) && (ptdata->y == ptdata2->y)) {
				// if the point hasn't moved at all, then we could be in a non-burning condition or the point has stopped, so remove redundancy by removing the following step
				polyset->Remove(poly2);
				polyset->Delete(poly2);
				poly2 = poly->LN_Succ();
			}						// this was a duplicate so look for another duplicate at the same spot
			else
				break;				// check the next point combo for possible duplicates
		}
		poly = poly->LN_Succ();
	}

	if (!(flags & 2)) {					// this bit says to leave in grid units, not convert to UTM (which is default)
	auto poly = polyset->LH_Head();
	while (poly->LN_Succ()) {
		auto pt = poly->LH_Head();
		while (pt->LN_Succ()) {
			fromInternal(*pt);
			pt = pt->LN_Succ();
		}
		poly = poly->LN_Succ();
	}
	}

	return S_OK;
}


template<class _type>
HRESULT Scenario<_type>::ExportCriticalPath(const AssetNode<_type>* node, const AssetGeometryNode<_type>* g, const std::uint16_t flags, const TCHAR* driver_name, const TCHAR* csProjection, const TCHAR* file_path, const ScenarioExportRules& rules) const {
	if (g) {
	if (!g->m_arrived)
		return ERROR_SCENARIO_ASSET_NOT_ARRIVED;
	}
	PolymorphicAttribute var;
	HRESULT hr;
	if (FAILED(hr = m_scenario->m_gridEngine->GetAttribute(nullptr, CWFGM_GRID_ATTRIBUTE_SPATIALREFERENCE, &var)))
		return hr;
	std::string projection = std::get<std::string>(var);

	OGRSpatialReferenceH oSourceSRS = CCoordinateConverter::CreateSpatialReferenceFromWkt(projection.c_str());
	OGRSpatialReferenceH oTargetSRS = CCoordinateConverter::CreateSpatialReferenceFromStr(csProjection);

	CriticalPath polyset;
	if (g) {
		if (g->m_arrived)
	BuildCriticalPath(node, g, 0, &polyset, &rules);
	} else {
		g = node->m_geometry.LH_Head();
		while (g->LN_Succ()) {
			if (g->m_arrived) {
				CriticalPath polyset2;
				BuildCriticalPath(node, g, 0, &polyset2, &rules);
				CriticalPathPoint* cpp;
				while (cpp = (CriticalPathPoint*)polyset2.RemHead())
					polyset.AddPoly(cpp);
			}
			g = g->LN_Succ();
		}
	}

	polyset.ExportPoly(driver_name, file_path, oSourceSRS, oTargetSRS);
	if (oSourceSRS)
		CCoordinateConverter::DestroySpatialReference(oSourceSRS);
	if (oTargetSRS)
		CCoordinateConverter::DestroySpatialReference(oTargetSRS);

	return S_OK;
}



IMPLEMENT_OBJECT_CACHE_MT_NO_TEMPLATE(CriticalPathPointData, CriticalPathPointData, 1024 * 1024 * 256 / sizeof(CriticalPathPointData), true, 16)
IMPLEMENT_OBJECT_CACHE_MT_NO_TEMPLATE(CriticalPathPoint, CriticalPathPoint, 4 * 1024 * 1024 / sizeof(CriticalPathPoint), false, 16)
IMPLEMENT_OBJECT_CACHE_MT_NO_TEMPLATE(CriticalPath, CriticalPath, 4 * 1024 * 1024 / sizeof(CriticalPath), false, 16)
IMPLEMENT_OBJECT_CACHE_MT_NO_TEMPLATE(ExportRule, ExportRule, 4 * 1024 * 1024 / sizeof(ExportRule), true, 16)


CriticalPathPointData::CriticalPathPointData(const CriticalPathPointData& c) : XY_PolyNodeAttribute<double>(c) {
}


CriticalPathPoint::CriticalPathPoint() : XY_PolyLLAttributes(12) {
}


CriticalPathPoint::CriticalPathPoint(const CriticalPathPoint& toCopy) : XY_PolyLLAttributes() {
	CriticalPathPointData* node = (CriticalPathPointData *)toCopy.LH_Head();
	while (node->LN_Succ()) {
		CriticalPathPointData* copy = new CriticalPathPointData(*node);
		m_ptList.AddTail(copy);
		node = node->LN_Succ();
	}
	size_t i, size = toCopy.m_attributes.size();
	m_attributes.reserve(size);
	for (i = 0; i < size; i++)
		m_attributes.push_back(toCopy.m_attributes[i]);
}


CriticalPathPoint::~CriticalPathPoint() {
	CriticalPathPointData* node;
	while (node = (CriticalPathPointData*)RemHead())
		delete node;
}


CriticalPathPointData* CriticalPathPoint::New() const {
	return new CriticalPathPointData();
}


CriticalPathPoint* CriticalPath::NewCopy(const XY_PolyLL_BaseTempl<double>& c) const {
	return new CriticalPathPoint((CriticalPathPoint &)c);
}



CriticalPathPointData* CriticalPathPoint::NewCopy(const XY_PolyLLNode<double>& toCopy) const {
	return new CriticalPathPointData((CriticalPathPointData &)toCopy);
}


void CriticalPathPoint::Delete(XY_PolyLLNode<double>* toDelete) const {
	delete (CriticalPathPointData*)toDelete;
}


CriticalPath::CriticalPath() : XY_PolyLLSetAttributes() {
}


CriticalPath::CriticalPath(const CriticalPath& toCopy) {
	CriticalPathPoint* node = (CriticalPathPoint*)toCopy.LH_Head();
	while (node->LN_Succ()) {
		CriticalPathPoint* copy = new CriticalPathPoint(*node);
		AddTail(copy);
		node = node->LN_Succ();
	}
}


CriticalPathPoint* CriticalPath::New() const {
	return new CriticalPathPoint();
}


void CriticalPath::Delete(XY_PolyLL_BaseTempl<double>* toDelete) const {
	delete (CriticalPathPoint *)toDelete;
}


CriticalPath::~CriticalPath() {
	CriticalPathPoint* node;
	while (node = (CriticalPathPoint*)RemHead())
		delete node;
}

#include "InstantiateClasses.cpp"
