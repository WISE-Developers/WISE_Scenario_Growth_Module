/**
 * WISE_Scenario_Growth_Module: CWFGM_Scenario.cpp
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
#include "FireEngine_ext.h"
#include "GridCom_ext.h"
#include "WeatherCom_ext.h"
#include "CWFGM_Scenario.h"
#include "scenario.h"
#include "ScenarioTimeStep.h"
#include "results.h"
#include "rectangles.h"
#include "ScenarioExportRules.h"
#include "excel_tinv.h"
#include "WinReplacement.h"
#include "CWFGM_Scenario_Internal.h"

#ifdef DEBUG
#include <cassert>
#endif


CCWFGM_Scenario::CCWFGM_Scenario() : m_timeManager(nullptr),
    m_startTime((std::uint64_t)0, m_timeManager),
    m_endTime((std::uint64_t)0, m_timeManager),
	m_ignitionOverrideTime((std::uint64_t)0, m_timeManager),
	m_impl{ std::make_unique<Impl>() }
{
	m_fwi = new ICWFGM_FWI;

	m_optionFlags = (1ull << CWFGM_SCENARIO_OPTION_ACCEL) | (1ull << CWFGM_SCENARIO_OPTION_BUI)
	    | (1ull << CWFGM_SCENARIO_OPTION_WIND) | (1ull << CWFGM_SCENARIO_OPTION_FMC_TERRAIN) | (1ull << CWFGM_SCENARIO_OPTION_TOPOGRAPHY)
	    | (1ull << CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMP_RH)
	    | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_PRECIP) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND) | (1ull << (CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND_VECTOR))
	    | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY)
	    | (1ull << CWFGM_SCENARIO_OPTION_BREACHING) | (1ull << CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC)
		| (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN) | (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)
		;

	m_perimeterResolution = 1.0;
	m_perimeterSpacing = 0.001;
	m_spatialThreshold = 1.0;
	m_minimumROS = 0.001;
	m_temporalThreshold_Accel = WTimeSpan(0, 0, 2, 0);
	m_displayInterval = WTimeSpan(0, 1, 0, 0);
	m_ignitionSize = 0.5;

	m_gridEngine = nullptr;
	m_bRequiresSave = false;

	m_initialVertexCount = 16;
	m_specifiedFMC = 120.0;
	m_defaultElevation = -99.0;

	m_layerThread = nullptr;

	m_threadingNumProcessors = CWorkerThreadPool::NumberProcessors();

	m_dx = m_dy = m_dwd = m_dvd = 0.0;
	m_owd = m_ovd = -1.0;
	m_dt = WTimeSpan(0LL, true);
	m_growthPercentile = 50.0;

	m_windTargetIndex = m_windTargetSubIndex = m_vectorTargetIndex = m_vectorTargetSubIndex = (ULONG)-1;
}


CCWFGM_Scenario::CCWFGM_Scenario(const CCWFGM_Scenario &toCopy) : m_timeManager(toCopy.m_timeManager),
	m_startTime((std::uint64_t)0, m_timeManager),
	m_endTime((std::uint64_t)0, m_timeManager),
	m_ignitionOverrideTime((std::uint64_t)0, m_timeManager),
	m_sp(toCopy.m_sp),
	m_impl{ std::make_unique<Impl>() }
{
	CRWThreadSemaphoreEngage engage(*(CRWThreadSemaphore *)&toCopy.m_lock, SEM_FALSE);

	m_fwi = new ICWFGM_FWI();

	m_startTime.SetTime(toCopy.m_startTime);
	m_endTime.SetTime(toCopy.m_endTime);
	m_ignitionOverrideTime.SetTime(toCopy.m_ignitionOverrideTime);

	m_optionFlags = toCopy.m_optionFlags;

#ifndef USE_BIGFLOATS
	m_optionFlags |= (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN) | (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING);
#endif			// if we aren't using 128-bit floats, then we are using 64-bit floats and we know we have issues in UTM space, so force this on

	m_perimeterResolution = toCopy.m_perimeterResolution;
	m_spatialThreshold = toCopy.m_spatialThreshold;
	m_minimumROS = toCopy.m_minimumROS;
	m_temporalThreshold_Accel = toCopy.m_temporalThreshold_Accel;
	m_displayInterval = toCopy.m_displayInterval;
	m_ignitionSize = toCopy.m_ignitionSize;
	m_gridEngine = toCopy.m_gridEngine;
	m_bRequiresSave = false;
	m_initialVertexCount = toCopy.m_initialVertexCount;
	m_specifiedFMC = toCopy.m_specifiedFMC;
	m_defaultElevation = toCopy.m_defaultElevation;
	m_layerThread = toCopy.m_layerThread;
	m_threadingNumProcessors = toCopy.m_threadingNumProcessors;
	m_dx = toCopy.m_dx;
	m_dy = toCopy.m_dy;
	m_dwd = toCopy.m_dwd;
	m_owd = toCopy.m_owd;
	m_dt = toCopy.m_dt;
	m_growthPercentile = toCopy.m_growthPercentile;
	m_sc = toCopy.m_sc;

	m_windTarget = toCopy.m_windTarget;
	m_windTargetIndex = toCopy.m_windTargetIndex;
	m_windTargetSubIndex = toCopy.m_windTargetSubIndex;
	m_vectorTarget = toCopy.m_vectorTarget;
	m_vectorTargetIndex = toCopy.m_vectorTargetIndex;
	m_vectorTargetSubIndex = toCopy.m_vectorTargetSubIndex;

	IgnitionNode<fireengine_float_type> *in = toCopy.m_impl->m_ignitionList.LH_Head();
	while (in->LN_Succ()) {
		IgnitionNode<fireengine_float_type> *cin = new IgnitionNode<fireengine_float_type>(*in, m_timeManager);
		m_impl->m_ignitionList.AddTail(cin);
		in = in->LN_Succ();
	}

	VectorEngineNode *vn = toCopy.m_vectorEngineList.LH_Head();
	while (vn->LN_Succ()) {
		VectorEngineNode *nvn = new VectorEngineNode();
		nvn->m_vectorEngine = vn->m_vectorEngine;
		m_vectorEngineList.AddTail(nvn);
		vn = vn->LN_Succ();
	}
}


CCWFGM_Scenario::~CCWFGM_Scenario() {
	IgnitionNode<fireengine_float_type> *fn;
	while (fn = m_impl->m_ignitionList.RemHead())
		delete fn;

	if (m_impl->m_scenario) {
		weak_assert(false);
		m_impl->unlockObjects(*this, NULL, (AssetNode<fireengine_float_type>*)~0);
		delete m_impl->m_scenario;
	}

	VectorEngineNode *vn;
	while (vn = m_vectorEngineList.RemHead())
		delete vn;
	weak_assert(!m_gridEngine);
	
	delete m_fwi;
}


HRESULT CCWFGM_Scenario::GetGridEngine(Layer **layerThread, boost::intrusive_ptr<ICWFGM_GridEngine> *pVal) const {
	if (!layerThread)								return E_POINTER;
	if (!pVal)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	*pVal = m_gridEngine;
	*layerThread = m_layerThread;
	if (!m_gridEngine)								{ weak_assert(false); return ERROR_GRID_UNINITIALIZED; }
	return S_OK;
}


HRESULT CCWFGM_Scenario::PutGridEngine(Layer *layerThread, ICWFGM_GridEngine *newVal) {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	HRESULT retval;
	if (newVal) {
		boost::intrusive_ptr<ICWFGM_GridEngine> pGridEngine;
		pGridEngine = dynamic_cast<ICWFGM_GridEngine *>(const_cast<ICWFGM_GridEngine *>(newVal));
		if (pGridEngine.get()) {
			m_gridEngine = pGridEngine;
			m_layerThread = layerThread;
			retval = S_OK;
		}
		else
			retval = E_FAIL;
	} else {
		m_layerThread = layerThread;
		m_gridEngine = newVal;
		retval = S_OK;
	}
	return retval;
}


HRESULT CCWFGM_Scenario::PutCommonData(Layer* layerThread, ICWFGM_CommonData* pVal) {
	if (!pVal)
		return E_POINTER;
	m_timeManager = pVal->m_timeManager;
	m_startTime.SetTimeManager(m_timeManager);
	m_endTime.SetTimeManager(m_timeManager);
	m_ignitionOverrideTime.SetTimeManager(m_timeManager);
	return S_OK;
}


HRESULT CCWFGM_Scenario::GetAttribute(std::uint16_t option,  PolymorphicAttribute *value) const {
	if (!value)									return E_POINTER;

	std::uint64_t mask;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	switch (option) {
	case CWFGM_SCENARIO_OPTION_FALSE_ORIGIN:
	case CWFGM_SCENARIO_OPTION_FALSE_SCALING:

#ifndef USE_BIGFLOATS
															return ERROR_SCENARIO_OPTION_INVALID;	// can't let these be toggled when using 64-bit floats
#endif

		case CWFGM_SCENARIO_OPTION_TOPOGRAPHY:
		case CWFGM_SCENARIO_OPTION_FMC_TERRAIN:
		case CWFGM_SCENARIO_OPTION_WIND:
		case CWFGM_SCENARIO_OPTION_EXTINGUISHMENT:
		case CWFGM_SCENARIO_OPTION_USE_2DGROWTH:
		case CWFGM_SCENARIO_OPTION_BOUNDARY_STOP:
		case CWFGM_SCENARIO_OPTION_SPOTTING:
		case CWFGM_SCENARIO_OPTION_BREACHING:
		case CWFGM_SCENARIO_OPTION_SINGLETHREADING:
		case CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMPORAL:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_PRECIP:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMP_RH:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY:
		case CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION:
		case CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC:
		case CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE_ENABLE:
		case CWFGM_SCENARIO_OPTION_CARDINAL_ROS:
		case CWFGM_SCENARIO_OPTION_CACHE_GRID_POINTS:
		case CWFGM_SCENARIO_OPTION_SPECIFIED_FMC_ACTIVE:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND_VECTOR:
		case CWFGM_SCENARIO_OPTION_FORCE_AFFINITY:
		case CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS:
		case CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT:
															mask = 1ull << option;
															*value = (m_optionFlags & mask) ? true : false;
															return S_OK;

		case CWFGM_SCENARIO_OPTION_MULTITHREADING:			*value = m_threadingNumProcessors;
															return S_OK;

		case CWFGM_SCENARIO_OPTION_PERIMETER_RESOLUTION:	*value = m_perimeterResolution;			return S_OK;
		case CWFGM_SCENARIO_OPTION_PERIMETER_SPACING:		*value = m_perimeterSpacing;			return S_OK;
		case CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD:		*value = m_spatialThreshold;			return S_OK;
		case CWFGM_SCENARIO_OPTION_MINIMUM_SPREADING_ROS:	*value = m_minimumROS;					return S_OK;
		case CWFGM_GRID_ATTRIBUTE_LATITUDE:					*value = m_timeManager->m_worldLocation.m_latitude();	return S_OK;
		case CWFGM_GRID_ATTRIBUTE_LONGITUDE:				*value = m_timeManager->m_worldLocation.m_longitude();	return S_OK;
		case CWFGM_SCENARIO_OPTION_SPECIFIED_FMC:			*value = m_specifiedFMC;				return S_OK;
		case CWFGM_SCENARIO_OPTION_DEFAULT_ELEVATION:		*value = m_defaultElevation;			return S_OK;
		case CWFGM_SCENARIO_OPTION_IGNITION_SIZE:			*value = m_ignitionSize;				return S_OK;


		case CWFGM_SCENARIO_OPTION_START_TIME:				*value = m_startTime;
															return S_OK;

		case CWFGM_SCENARIO_OPTION_END_TIME:				*value = m_endTime;
															return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITION_START_TIME_OVERRIDE:
															*value = m_ignitionOverrideTime;
															return S_OK;

		case CWFGM_SCENARIO_OPTION_CURRENT_TIME:			if (m_impl->m_scenario) {
																*value = m_impl->m_scenario->CurrentTime();
															} else	*value = m_startTime;
															return S_OK;

		case CWFGM_SCENARIO_OPTION_TEMPORAL_THRESHOLD_ACCEL:*value = m_temporalThreshold_Accel;
															return S_OK;

		case CWFGM_SCENARIO_OPTION_DISPLAY_INTERVAL:		*value = m_displayInterval;
															return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITIONS_DX:			*value = m_dx; return S_OK;
		case CWFGM_SCENARIO_OPTION_IGNITIONS_DY:			*value = m_dy; return S_OK;
		case CWFGM_SCENARIO_OPTION_IGNITIONS_DT:			*value = m_dt; return S_OK;
		case CWFGM_SCENARIO_OPTION_IGNITIONS_DWD:			*value = m_dwd; return S_OK;
		case CWFGM_SCENARIO_OPTION_IGNITIONS_OWD:			*value = m_owd; return S_OK;
		case CWFGM_SCENARIO_OPTION_GRID_DVD:				*value = m_dvd; return S_OK;
		case CWFGM_SCENARIO_OPTION_GRID_OVD:				*value = m_ovd; return S_OK;
		case CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE:	*value = m_growthPercentile; return S_OK;
		case CWFGM_ATTRIBUTE_LOAD_WARNING: {
							*value = m_loadWarning;
							return S_OK;
						   }
	}
	return ERROR_SCENARIO_OPTION_INVALID;
}


HRESULT CCWFGM_Scenario::SetAttribute(std::uint16_t option, const PolymorphicAttribute &value) {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);

	if (m_impl->m_scenario) {
		if (option != CWFGM_SCENARIO_OPTION_SINGLETHREADING)
			return ERROR_SCENARIO_BAD_STATE;
	}

	std::uint64_t mask;
	std::uint32_t l32Value;
	bool bValue;
	double dValue;
	WTimeSpan tsValue;
	WTime tValue((uint64_t)0, m_timeManager);
	std::uint64_t newVal;
	HRESULT hr;

	switch (option) {
		case CWFGM_SCENARIO_OPTION_TOPOGRAPHY:
		case CWFGM_SCENARIO_OPTION_FMC_TERRAIN:
		case CWFGM_SCENARIO_OPTION_WIND:
		case CWFGM_SCENARIO_OPTION_EXTINGUISHMENT:
		case CWFGM_SCENARIO_OPTION_USE_2DGROWTH:
		case CWFGM_SCENARIO_OPTION_BOUNDARY_STOP:
		case CWFGM_SCENARIO_OPTION_SPOTTING:
		case CWFGM_SCENARIO_OPTION_BREACHING:
		case CWFGM_SCENARIO_OPTION_SINGLETHREADING:
		case CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMPORAL:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_PRECIP:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMP_RH:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY:
		case CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION:
		case CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC:
		case CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE_ENABLE:
		case CWFGM_SCENARIO_OPTION_CARDINAL_ROS:
		case CWFGM_SCENARIO_OPTION_CACHE_GRID_POINTS:
		case CWFGM_SCENARIO_OPTION_SPECIFIED_FMC_ACTIVE:
		case CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND_VECTOR:
		case CWFGM_SCENARIO_OPTION_FORCE_AFFINITY:
		case CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS:
		case CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT:
		case CWFGM_SCENARIO_OPTION_FALSE_ORIGIN:
		case CWFGM_SCENARIO_OPTION_FALSE_SCALING:
								if (FAILED(hr = VariantToBoolean_(value, &bValue)))	return hr;
								mask = 1ull << option;
								if (bValue)		m_optionFlags |= mask;
								else			m_optionFlags &= (~mask);
								m_bRequiresSave = true;
								return S_OK;

		case CWFGM_SCENARIO_OPTION_MULTITHREADING:
								if (FAILED(hr = VariantToUInt64_(value, &mask)))	return hr;
								if (mask < 1)											return E_INVALIDARG;
								if (mask > (2 * CWorkerThreadPool::NumberProcessors()))	return E_INVALIDARG;
								m_threadingNumProcessors = mask;
								m_bRequiresSave = true;
								return S_OK;

		case CWFGM_SCENARIO_OPTION_PERIMETER_RESOLUTION:
								if (FAILED(hr = VariantToDouble_(value, &dValue)))	return hr;
								if (dValue < 0.2)		return E_INVALIDARG;
								if (dValue > 10.0)		return E_INVALIDARG;
								m_perimeterResolution = dValue;
								m_bRequiresSave = true;
								return S_OK;

		case CWFGM_SCENARIO_OPTION_PERIMETER_SPACING:
								if (FAILED(hr = VariantToDouble_(value, &dValue)))	return hr;
								if (dValue < 0.0)		return E_INVALIDARG;
								if (dValue > 10.0)		return E_INVALIDARG;
								m_perimeterSpacing = dValue;
								m_bRequiresSave = true;
								return S_OK;

		case CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD:
								if (FAILED(hr = VariantToDouble_(value, &dValue)))	return hr;
								if (dValue < 0.2)		return E_INVALIDARG;
								if (dValue > 10.0)		return E_INVALIDARG;
								m_spatialThreshold = dValue;
								m_bRequiresSave = true;
								return S_OK;

		case CWFGM_SCENARIO_OPTION_MINIMUM_SPREADING_ROS:
								if (FAILED(hr = VariantToDouble_(value, &dValue)))	return hr;
								if (dValue < 0.0000001)		return E_INVALIDARG;
								if (dValue > 1.0)		return E_INVALIDARG;
								m_minimumROS = dValue;
								m_bRequiresSave = true;
								return S_OK;

		case CWFGM_SCENARIO_OPTION_SPECIFIED_FMC:
								if (FAILED(hr = VariantToDouble_(value, &dValue)))			return hr;
								if (dValue < 0.0)										{ weak_assert(false); return E_INVALIDARG; }
								if (dValue > 300.0)										{ weak_assert(false); return E_INVALIDARG; }
								m_specifiedFMC = dValue;
								m_bRequiresSave = true;
								return S_OK;

		case CWFGM_SCENARIO_OPTION_DEFAULT_ELEVATION:
								if (FAILED(hr = VariantToDouble_(value, &dValue)))			return hr;
								if (dValue < 0.0) if ((dValue != -99.0) && (dValue != -1.0))		{ weak_assert(false); return E_INVALIDARG; }
								if (dValue > 7000.0)							{ weak_assert(false); return E_INVALIDARG; }
								m_defaultElevation = dValue;
								m_bRequiresSave = true;
								return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITION_SIZE:
								if (FAILED(hr = VariantToDouble_(value, &dValue)))			return hr;
								if (dValue <= 0.0)											return E_INVALIDARG;
								if (dValue > 25.0)											return E_INVALIDARG;
								m_ignitionSize = dValue;
								m_bRequiresSave = true;
								return S_OK;


		case CWFGM_SCENARIO_OPTION_START_TIME:
									if (FAILED(hr = VariantToTime_(value, &tValue)))		return hr;
									if ((!m_timeManager) && (tValue.GetTimeManager())) {
										m_timeManager = (WTimeManager*)tValue.GetTimeManager();	// needed to kickstart the default blank scenario needed
																								// in Proemtheus which coudl be more of a clean-up for WISE
										m_startTime.SetTimeManager(tValue.GetTimeManager());
										m_endTime.SetTimeManager(tValue.GetTimeManager());
										m_ignitionOverrideTime.SetTimeManager(tValue.GetTimeManager());
									}
									if (!tValue.GetTotalMicroSeconds())						return ERROR_SCENARIO_BAD_TIMES;
									m_startTime = tValue;
									m_startTime.PurgeToSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);

									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_END_TIME:
									if (FAILED(hr = VariantToTime_(value, &tValue)))		return hr;
									if (!tValue.GetTotalMicroSeconds())							return ERROR_SCENARIO_BAD_TIMES;
									m_endTime = tValue;
									m_endTime.PurgeToSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);

									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITION_START_TIME_OVERRIDE:
									if (FAILED(hr = VariantToTime_(value, &tValue)))			return hr;
									if (!tValue.GetTotalMicroSeconds())							return ERROR_SCENARIO_BAD_TIMES;

									m_ignitionOverrideTime = tValue;
									m_ignitionOverrideTime.PurgeToSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);

									return S_OK;

		case CWFGM_SCENARIO_OPTION_TEMPORAL_THRESHOLD_ACCEL:
									if (FAILED(hr = VariantToTimeSpan_(value, &tsValue)))		return hr;
									if ((tsValue.GetTotalSeconds() < 0) && (tsValue.GetTotalSeconds() != -1))	return ERROR_SCENARIO_BAD_TIMESTEPS;
									m_temporalThreshold_Accel = tsValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_DISPLAY_INTERVAL:
		                            if (FAILED(hr = VariantToTimeSpan_(value, &tsValue)))		return hr;
									if (tsValue.GetTotalSeconds() < 0)					return ERROR_SCENARIO_BAD_TIMESTEPS;
									m_displayInterval = tsValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITIONS_DX:
									if (FAILED(hr = VariantToDouble_(value, &dValue)))		return hr;
									if ((dValue < -250.0) || (dValue > 250.0))			{ weak_assert(false); return E_INVALIDARG; }
									m_dx = dValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITIONS_DY:
									if (FAILED(hr = VariantToDouble_(value, &dValue)))		return hr;
									if ((dValue < -250.0) || (dValue > 250.0))			{ weak_assert(false); return E_INVALIDARG; }
									m_dy = dValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITIONS_DT:
									if (FAILED(hr = VariantToTimeSpan_(value, &tsValue)))		return hr;
									if (tsValue > WTimeSpan(4 * 60 * 60))				return E_INVALIDARG;
									if (tsValue < WTimeSpan(-4 * 60 * 60))				return E_INVALIDARG;
									m_dt = tsValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITIONS_DWD:
									if (FAILED(hr = VariantToDouble_(value, &dValue)))		return hr;
									if ((dValue < -360.0) || (dValue > 360.0))			{ weak_assert(false); return E_INVALIDARG; }
									m_dwd = dValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITIONS_OWD:					if (FAILED(hr = VariantToDouble_(value, &dValue)))		return hr;
									if (((dValue != -1.0) && (dValue < 0.0)) || (dValue > 360.0)) { weak_assert(false); return E_INVALIDARG; }
									m_owd = dValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_GRID_DVD:						if (FAILED(hr = VariantToDouble_(value, &dValue)))		return hr;
									if ((dValue < -360.0) || (dValue > 360.0))			{ weak_assert(false); return E_INVALIDARG; }
									m_dvd = dValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_GRID_OVD:						if (FAILED(hr = VariantToDouble_(value, &dValue)))		return hr;
									if (((dValue != -1.0) && (dValue < 0.0)) || (dValue > 360.0)) { weak_assert(false); return E_INVALIDARG; }
									m_ovd = dValue;
									m_bRequiresSave = true;
									return S_OK;

		case CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE:
									if (FAILED(hr = VariantToDouble_(value, &dValue)))		return hr;

									if ((dValue <= 0.0) || (dValue >= 100.0))				return E_INVALIDARG;
									m_growthPercentile = dValue;
									m_bRequiresSave = true;
									return S_OK;
	}
	return ERROR_SCENARIO_OPTION_INVALID;
}


HRESULT CCWFGM_Scenario::AddIgnition(CCWFGM_Ignition *fire) {
	if (!fire)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	IgnitionNode<fireengine_float_type> *node = m_impl->m_ignitionList.LH_Head();
	while (node->LN_Succ()) {
		if (node->m_ignitionCOM == fire)
			return ERROR_SCENARIO_FIRE_ALREADY_ADDED;
		node = node->LN_Succ();
	}

	boost::intrusive_ptr<CCWFGM_Ignition> pFire;
	pFire = dynamic_cast<CCWFGM_Ignition *>(const_cast<CCWFGM_Ignition *>(fire));
	if (pFire) {
		try {
			node = new IgnitionNode<fireengine_float_type>(m_timeManager);
			node->m_ignitionCOM = pFire;
			m_impl->m_ignitionList.AddTail(node);
			m_bRequiresSave = true;
		} catch (std::exception &) {
			return E_OUTOFMEMORY;
		}
	}
	return S_OK;
}


HRESULT CCWFGM_Scenario::GetIgnitionCount(std::uint32_t *count) const {
	if (!count)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	*count = m_impl->m_ignitionList.GetCount();
	return S_OK;
}


HRESULT CCWFGM_Scenario::RemoveIgnition(CCWFGM_Ignition *fire) {
	if (!fire)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	IgnitionNode<fireengine_float_type> *node = m_impl->m_ignitionList.LH_Head();
	while (node->LN_Succ()) {
		if (node->m_ignitionCOM == fire) {
			m_impl->m_ignitionList.Remove(node);
			delete node;
			m_bRequiresSave = true;
			return S_OK;
		}
		node = node->LN_Succ();
	}
	return ERROR_SCENARIO_FIRE_UNKNOWN;
}


HRESULT CCWFGM_Scenario::IgnitionAtIndex(std::uint32_t index, boost::intrusive_ptr<CCWFGM_Ignition> *fire) const {
	if (!fire)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (index >= m_impl->m_ignitionList.GetCount())						return ERROR_SCENARIO_FIRE_UNKNOWN;
	IgnitionNode<fireengine_float_type> *node = m_impl->m_ignitionList.IndexNode(index);
	*fire = node->m_ignitionCOM;
	return S_OK;
}


HRESULT CCWFGM_Scenario::SetAssetOperation(ICWFGM_Asset* asset, std::uint32_t operation) {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	if (asset) {
		AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
		while (node->LN_Succ()) {
			if (node->m_asset == asset) {
				node->m_operation = operation;
				m_bRequiresSave = true;
				return S_OK;
			}
			node = node->LN_Succ();
		}
		return ERROR_SCENARIO_ASSET_UNKNOWN;
	}
	m_globalAssetOperation = operation;
	return S_OK;
}


HRESULT CCWFGM_Scenario::GetAssetOperation(ICWFGM_Asset* asset, std::uint32_t* operation) const {
	if (!operation)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	if (asset) {
		AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
		while (node->LN_Succ()) {
			if ((node->m_asset == asset) || ((m_impl->m_assetList.GetCount() == 1))) {
				*operation = node->m_operation;
				return S_OK;
			}
			node = node->LN_Succ();
		}
		return ERROR_SCENARIO_ASSET_UNKNOWN;
	}
	*operation = m_globalAssetOperation;
	return S_OK;
}


HRESULT CCWFGM_Scenario::GetAssetTimeCount(ICWFGM_Asset* asset, std::uint32_t* count) const {
	if (!count)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);
	if (!m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;
	if ((m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_ASSET) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI90) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI95) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI100) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_RH) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_PRECIP) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_AREA) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_BURNDISTANCE))
		return ERROR_SCENARIO_BAD_STATE;

	AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
	while (node->LN_Succ()) {
		if ((node->m_asset == asset) || ((asset == nullptr) && (m_impl->m_assetList.GetCount() == 1))) {
			*count = node->m_geometry.GetCount();
			return S_OK;
		}
		node = node->LN_Succ();
	}
	return ERROR_SCENARIO_ASSET_UNKNOWN;
}


HRESULT CCWFGM_Scenario::GetAssetTime(const ICWFGM_Asset* asset, const std::uint32_t index, bool* arrived, WTime* time) const {
	if (!arrived)								return E_POINTER;
	if (!time)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (!m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;
	if ((m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_ASSET) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI90) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI95) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI100) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_RH) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_PRECIP) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_AREA) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_BURNDISTANCE))
		return ERROR_SCENARIO_BAD_STATE;

	AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
	while (node->LN_Succ()) {
		if ((node->m_asset == asset) || ((asset == NULL) && (m_impl->m_assetList.GetCount() == 1))) {
			AssetGeometryNode<fireengine_float_type>* g = node->m_geometry.IndexNode(index);
			if (!g)
				return ERROR_SCENARIO_ASSET_GEOMETRY_UNKNOWN;
			*arrived = g->m_arrived;
			if (*arrived) {
				*time = WTime(g->m_arrivalTime, time->GetTimeManager());
				return S_OK;
			} else {
				*time = WTime((std::uint64_t)0, time->GetTimeManager());
				return ERROR_SCENARIO_ASSET_NOT_ARRIVED;
			}
		}
		node = node->LN_Succ();
	}
	*arrived = false;
	*time = WTime((std::uint64_t)0, time->GetTimeManager());
	return ERROR_SCENARIO_ASSET_UNKNOWN;
}


HRESULT CCWFGM_Scenario::IndexOfIgnition(const CCWFGM_Ignition *fire, std::uint32_t *index) const {
	if (!fire)									return E_POINTER;
	if (!index)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	IgnitionNode<fireengine_float_type> *node = m_impl->m_ignitionList.LH_Head();
	std::uint32_t i = 0;
	while (node->LN_Succ()) {
		if (node->m_ignitionCOM == fire) {
			*index = i;
			return S_OK;
		}
		node = node->LN_Succ();
		i++;
	}
	*index = (std::uint32_t)-1;
	return ERROR_SCENARIO_FIRE_UNKNOWN;
}


HRESULT CCWFGM_Scenario::AddVectorEngine(ICWFGM_VectorEngine *vectorEngine) {
	if (!vectorEngine)								return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	VectorEngineNode *node = m_vectorEngineList.LH_Head();
	while (node->LN_Succ()) {
		if (node->m_vectorEngine == vectorEngine)
			return ERROR_SCENARIO_VECTORENGINE_ALREADY_ADDED;
		node = node->LN_Succ();
	}

	boost::intrusive_ptr<ICWFGM_VectorEngine> pVectorEngine;
	pVectorEngine = dynamic_cast<ICWFGM_VectorEngine *>(const_cast<ICWFGM_VectorEngine *>(vectorEngine));
	if (pVectorEngine.get()) {
		try {
			node = new VectorEngineNode();
			node->m_vectorEngine = pVectorEngine;
			m_vectorEngineList.AddTail(node);
			m_bRequiresSave = true;
		} catch (std::exception &e) {
			return E_OUTOFMEMORY;
		}
	}
	return S_OK;
}


HRESULT CCWFGM_Scenario::AddAsset(ICWFGM_Asset* asset, std::uint32_t operation) {
	if (!asset)								return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
	while (node->LN_Succ()) {
		if (node->m_asset == asset)
			return ERROR_SCENARIO_ASSET_ALREADY_ADDED;
		node = node->LN_Succ();
	}

	boost::intrusive_ptr<ICWFGM_Asset> pAsset;
	pAsset = dynamic_cast<ICWFGM_Asset*>(const_cast<ICWFGM_Asset*>(asset));
	if (pAsset.get()) {
		try {
			node = new AssetNode<fireengine_float_type>();
			node->m_asset = pAsset;
			node->m_operation = operation;
			m_impl->m_assetList.AddTail(node);
			m_bRequiresSave = true;
		} catch (std::exception& e) {
			return E_OUTOFMEMORY;
		}
	}
	return S_OK;
}


HRESULT CCWFGM_Scenario::GetVectorEngineCount(std::uint32_t *count) const {
	if (!count)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	*count = m_vectorEngineList.GetCount();
	return S_OK;
}


HRESULT CCWFGM_Scenario::GetAssetCount(std::uint32_t* count) const {
	if (!count)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	*count = m_impl->m_assetList.GetCount();
	return S_OK;
}


HRESULT CCWFGM_Scenario::RemoveVectorEngine(ICWFGM_VectorEngine *vectorEngine) {
	if (!vectorEngine)								return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	VectorEngineNode *node = m_vectorEngineList.LH_Head();
	while (node->LN_Succ()) {
		if (node->m_vectorEngine == vectorEngine) {
			m_vectorEngineList.Remove(node);
			delete node;
			m_bRequiresSave = true;
			return S_OK;
		}
		node = node->LN_Succ();
	}
	return ERROR_SCENARIO_VECTORENGINE_UNKNOWN;
}


HRESULT CCWFGM_Scenario::RemoveAsset(ICWFGM_Asset* asset) {
	if (!asset)								return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
	while (node->LN_Succ()) {
		if (node->m_asset == asset) {
			m_vectorEngineList.Remove(node);
			delete node;
			m_bRequiresSave = true;
			return S_OK;
		}
		node = node->LN_Succ();
	}
	return ERROR_SCENARIO_ASSET_UNKNOWN;
}


HRESULT CCWFGM_Scenario::SetWindTarget(ICWFGM_Target* target, unsigned long index, unsigned long sub_index) {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	if (target) {
		boost::intrusive_ptr<ICWFGM_Target> pTarget;
		pTarget = dynamic_cast<ICWFGM_Target*>(const_cast<ICWFGM_Target*>(target));
		if (!pTarget.get())
			return E_NOINTERFACE;
	}

	m_windTarget = target;
	m_windTargetIndex = index;
	m_windTargetSubIndex = sub_index;

	return S_OK;
}


HRESULT CCWFGM_Scenario::GetWindTarget(ICWFGM_Target** target, unsigned long* index, unsigned long* sub_index) const {
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	*target = m_windTarget.get();
	*index = m_windTargetIndex;
	*sub_index = m_windTargetSubIndex;

	return S_OK;
}


HRESULT CCWFGM_Scenario::ClearWindTarget() {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	m_windTarget = NULL;
	m_windTargetIndex = (ULONG)-1;
	m_windTargetSubIndex = (ULONG)-1;

	return S_OK;
}


HRESULT CCWFGM_Scenario::SetVectorTarget(ICWFGM_Target* target, unsigned long index, unsigned long sub_index) {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	if (target) {
		boost::intrusive_ptr<ICWFGM_Target> pTarget;
		pTarget = dynamic_cast<ICWFGM_Target*>(const_cast<ICWFGM_Target*>(target));
		if (!pTarget.get())
			return E_NOINTERFACE;
	}

	m_vectorTarget = target;
	m_vectorTargetIndex = index;
	m_vectorTargetSubIndex = sub_index;

	return S_OK;
}


HRESULT CCWFGM_Scenario::GetVectorTarget(ICWFGM_Target** target, unsigned long* index, unsigned long* sub_index) {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_FALSE);

	*target = m_vectorTarget.get();
	*index = m_vectorTargetIndex;
	*sub_index = m_vectorTargetSubIndex;

	return S_OK;
}


HRESULT CCWFGM_Scenario::ClearVectorTarget() {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	m_vectorTarget = nullptr;
	m_vectorTargetIndex = (ULONG)-1;
	m_vectorTargetSubIndex = (ULONG)-1;

	return S_OK;
}


HRESULT CCWFGM_Scenario::VectorEngineAtIndex(std::uint32_t index, boost::intrusive_ptr<ICWFGM_VectorEngine> *vectorEngine) const {
	if (!vectorEngine)								return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);
	if (index >= m_vectorEngineList.GetCount())					return ERROR_SCENARIO_VECTORENGINE_UNKNOWN;
	VectorEngineNode *node = m_vectorEngineList.IndexNode(index);
	*vectorEngine = node->m_vectorEngine;
	return S_OK;
}


HRESULT CCWFGM_Scenario::AssetAtIndex(const std::uint32_t index, boost::intrusive_ptr<ICWFGM_Asset>* asset) const {
	if (!asset)								return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);
	if (index >= m_impl->m_assetList.GetCount())					return ERROR_SCENARIO_VECTORENGINE_UNKNOWN;
	AssetNode<fireengine_float_type>* node = m_impl->m_assetList.IndexNode(index);
	*asset = node->m_asset;
	return S_OK;
}


HRESULT CCWFGM_Scenario::IndexOfVectorEngine(const ICWFGM_VectorEngine *vectorEngine, std::uint32_t *index) const {
	if (!vectorEngine)								return E_POINTER;
	if (!index)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	VectorEngineNode *node = m_vectorEngineList.LH_Head();
	std::uint32_t i = 0;
	while (node->LN_Succ()) {
		if (node->m_vectorEngine == vectorEngine) {
			*index = i;
			return S_OK;
		}
		node = node->LN_Succ();
		i++;
	}
	*index = (std::uint32_t)-1;
	return ERROR_SCENARIO_VECTORENGINE_UNKNOWN;
}


HRESULT CCWFGM_Scenario::IndexOfAsset(const ICWFGM_Asset* asset, std::uint32_t* index) const {
	if (!asset)									return E_POINTER;
	if (!index)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
	std::uint32_t i = 0;
	while (node->LN_Succ()) {
		if (node->m_asset == asset) {
			*index = i;
			return S_OK;
		}
		node = node->LN_Succ();
		i++;
	}
	*index = (std::uint32_t) - 1;
	return ERROR_SCENARIO_ASSET_UNKNOWN;
}


HRESULT CCWFGM_Scenario::Clone(boost::intrusive_ptr<ICWFGM_CommonBase> *newObject) const {
	if (!newObject)							return E_POINTER;

	CRWThreadSemaphoreEngage engage(*(CRWThreadSemaphore *)&m_lock, SEM_FALSE);

	try {
		CCWFGM_Scenario *f = new CCWFGM_Scenario(*this);
		*newObject = f;
		return S_OK;
	}
	catch (std::exception &e) {
	}
	return E_FAIL;
}


HRESULT CCWFGM_Scenario::Simulation_State() {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_FALSE);
	if (m_impl->m_scenario) {
		CRWThreadSemaphoreEngage _semaphore_engage2(m_impl->m_scenario->m_llLock, SEM_FALSE);
		if (!m_impl->m_scenario->m_timeSteps.GetCount())						return SUCCESS_SCENARIO_SIMULATION_RESET;

		ScenarioTimeStep<fireengine_float_type> *sts = m_impl->m_scenario->m_timeSteps.LH_Tail();
		if (sts->m_lock.CurrentState() < 0)										return SUCCESS_SCENARIO_SIMULATION_RUNNING;

		if (sts->m_time == m_endTime)											return SUCCESS_SCENARIO_SIMULATION_COMPLETE;
		if (m_impl->m_scenario->m_stepState == E_OUTOFMEMORY)					return SUCCESS_SCENARIO_SIMULATION_COMPLETE;
		if (m_impl->m_scenario->m_stepState == S_OK)							return SUCCESS_SCENARIO_SIMULATION_RUNNING;
		return m_impl->m_scenario->m_stepState;
	}

	if (!m_gridEngine)										  { weak_assert(false); return ERROR_GRID_UNINITIALIZED; }
	if (m_impl->m_ignitionList.IsEmpty())										return ERROR_SCENARIO_NO_FIRES;
	if (!m_startTime.GetTime(0))												return ERROR_SCENARIO_BAD_TIMES;
	if (!m_endTime.GetTime(0))													return ERROR_SCENARIO_BAD_TIMES;
	if (m_endTime <= m_startTime)												return ERROR_SCENARIO_BAD_TIMES;

	bool foundTimezone = false;
	WTimeSpan time;
	WTimeSpan ts = m_endTime - m_startTime;
	HRESULT hr;
	if (FAILED(hr = m_gridEngine->Valid(m_layerThread, m_startTime, ts, 0, nullptr)))	return hr;

	VectorEngineNode *ven = m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		if (FAILED(hr = ven->m_vectorEngine->Valid(m_startTime, ts))) {
			return hr;
		}
		ven = ven->LN_Succ();
	}

	AssetNode<fireengine_float_type>* an = m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		if (FAILED(hr = an->m_asset->Valid(m_startTime, ts))) {
			return hr;
		}
		an = an->LN_Succ();
	}

	IgnitionNode<fireengine_float_type> *ie = m_impl->m_ignitionList.LH_Head();
	while (ie->LN_Succ()) {
		if (FAILED(hr = ie->m_ignitionCOM->Valid(m_startTime, ts))) {
			return hr;
		}
		ie = (IgnitionNode<fireengine_float_type>*)ie->LN_Succ();
	}

	m_gridEngine->PreCalculationEvent(m_layerThread, m_startTime, 0, nullptr);
	m_gridEngine->PreCalculationEvent(m_layerThread, m_startTime, 1, nullptr);

	ven = m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->PreCalculationEvent(m_startTime, 0, nullptr);
		ven->m_vectorEngine->PreCalculationEvent(m_startTime, 1, nullptr);
		ven = ven->LN_Succ();
	}
	an = m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->PreCalculationEvent(m_startTime, 0, nullptr);
		an->m_asset->PreCalculationEvent(m_startTime, 1, nullptr);
		an = an->LN_Succ();
	}

	PolymorphicAttribute var;
	if (!m_timeManager) {
		weak_assert(false);
		ICWFGM_CommonData* data;
		if (FAILED(hr = m_gridEngine->GetCommonData(m_layerThread, &data)) || (!data))							goto DONE;
		m_timeManager = data->m_timeManager;
	}
	double temp;
	hr = S_OK;

DONE:
	an = m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->PostCalculationEvent(m_endTime, 1, nullptr);
		an->m_asset->PostCalculationEvent(m_endTime, 0, nullptr);
		an = an->LN_Succ();
	}
	ven = m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->PostCalculationEvent(m_endTime, 1, nullptr);
		ven->m_vectorEngine->PostCalculationEvent(m_endTime, 0, nullptr);
		ven = ven->LN_Succ();
	}

	m_gridEngine->PostCalculationEvent(m_layerThread, m_endTime, 1, nullptr);
	m_gridEngine->PostCalculationEvent(m_layerThread, m_endTime, 0, nullptr);
	return hr;
}


HRESULT CCWFGM_Scenario::lockObjects() {
	HRESULT hr;
	if (FAILED(hr = m_gridEngine->MT_Lock(m_layerThread, false, 1)))						return hr;
	
	IgnitionNode<fireengine_float_type> *n = m_impl->m_ignitionList.LH_Head();
	while (n->LN_Succ()) {
		if (FAILED(hr = n->m_ignitionCOM->MT_Lock(false, 1))) {
			IgnitionNode<fireengine_float_type> *nn = m_impl->m_ignitionList.LH_Head();
			while (nn != n) {
				nn->m_ignitionCOM->MT_Lock(false, 0);
				nn = nn->LN_Succ();
			}
			m_gridEngine->MT_Lock(m_layerThread, false, 0);
			return hr;
		}
		n = n->LN_Succ();
	}
	VectorEngineNode *ve = m_vectorEngineList.LH_Head();
	while (ve->LN_Succ()) {
		if (FAILED(hr = ve->m_vectorEngine->MT_Lock(false, 1))) {
			m_impl->unlockObjects(*this, ve, (AssetNode<fireengine_float_type>*)nullptr);
			return hr;
		}
		ve = ve->LN_Succ();
	}
	AssetNode<fireengine_float_type>* an = m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		if (FAILED(hr = an->m_asset->MT_Lock(false, 1))) {
			m_impl->unlockObjects(*this, nullptr, an);
			return hr;
		}
		an = an->LN_Succ();
	}
	return hr;
}


void CCWFGM_Scenario::Impl::unlockObjects(CCWFGM_Scenario& _this, VectorEngineNode *ve_stop, AssetNode<fireengine_float_type>* an_stop) {
	if (an_stop) {
		AssetNode<fireengine_float_type>* an = m_assetList.LH_Head();
		while (an->LN_Succ()) {
			if (an == an_stop)
				break;
			an->m_asset->MT_Lock(false, 0);
			an = an->LN_Succ();
		}
	}
	VectorEngineNode *ve = _this.m_vectorEngineList.LH_Head();
	while (ve->LN_Succ()) {
		if (ve == ve_stop)
			break;
		ve->m_vectorEngine->MT_Lock(false, 0);
		ve = ve->LN_Succ();
	}
	IgnitionNode<fireengine_float_type> *n = m_ignitionList.LH_Head();
	while (n->LN_Succ()) {
		n->m_ignitionCOM->MT_Lock(false, 0);
		n = n->LN_Succ();
	}
	_this.m_gridEngine->MT_Lock(_this.m_layerThread, false, 0);
}


HRESULT CCWFGM_Scenario::checkWxGridObject(const WTimeSpan &duration, std::uint32_t option, std::vector<uint16_t> *s) {
	HRESULT hr = m_gridEngine->Valid(m_layerThread, m_startTime, duration, option, s);
	if (FAILED(hr))
		return S_OK;									// no objects responded to option so it will pass
	for (std::int64_t i = 0; i < duration.GetTotalSeconds(); i++)
		if ((*s)[i] > 1) {
			if (option == CWFGM_WEATHER_WXGRID_WS_DIURNALTIMES)
				return ERROR_SCENARIO_MULTIPLE_WS_GRIDS;
			else if (option == CWFGM_WEATHER_WXGRID_WD_DIURNALTIMES)
				return ERROR_SCENARIO_MULTIPLE_WD_GRIDS;
			else	return E_FAIL;
		}
	return S_OK;
}


HRESULT CCWFGM_Scenario::checkWxGridObjects() {
	WTimeSpan size = m_endTime - m_startTime + WTimeSpan(1);
	try {
		HRESULT hr;
		std::vector<uint16_t> s(size.GetTotalSeconds());
		for (std::uint64_t i = 0; i < size.GetTotalSeconds(); i++)
			s[i] = 0;
		if (SUCCEEDED(hr = checkWxGridObject(size, CWFGM_WEATHER_WXGRID_WS_DIURNALTIMES, &s))) {
			for (std::uint64_t i = 0; i < size.GetTotalSeconds(); i++)
				s[i] = 0;
			hr = checkWxGridObject(size, CWFGM_WEATHER_WXGRID_WD_DIURNALTIMES, &s);
		}
		return hr;
	} catch (std::bad_alloc &cme) {
		return E_OUTOFMEMORY;
	}
}


HRESULT CCWFGM_Scenario::Simulation_Reset(std::shared_ptr<validation::validation_object> valid, const std::string& name) {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;
	if (!m_gridEngine) {
		if (valid)
			valid->add_child_validation("WISE.FireEngineProto.CwfgmScenario", name, validation::error_level::WARNING, validation::id::initialization_incomplete, "grid_engine");
		weak_assert(false);
		return ERROR_GRID_UNINITIALIZED;
	}

	auto vt = validation::conditional_make_object(valid, "WISE.FireEngineProto.CwfgmScenario", name);
	auto v = vt.lock();

	if (m_impl->m_scenario) {
		if (v)
			/// <summary>
			/// The scenario can't be reset because it's currently running.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("Simulation:Reset", "state", validation::error_level::SEVERE, validation::id::state_invalid, "", "Scenario can't be reset when it's running.");
		return ERROR_SCENARIO_BAD_STATE;
	}
	if (m_impl->m_ignitionList.IsEmpty()) {
		if (v)
			/// <summary>
			/// The scenario can't be reset because it's currently running.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("Simulation:Reset", "ignitions", validation::error_level::SEVERE, validation::id::scenario_missing_ignitions, "", "");
		return ERROR_SCENARIO_NO_FIRES;
	}
	if (!m_startTime.GetTime(0)) {
		if (v)
			/// <summary>
			/// The scenario start time is out of range or invalid.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTime", "startTime", validation::error_level::WARNING, validation::id::time_invalid, m_startTime.ToString(WTIME_FORMAT_STRING_ISO8601), { true, WTime::GlobalMin().ToString(WTIME_FORMAT_STRING_ISO8601) }, { true, WTime::GlobalMax().ToString(WTIME_FORMAT_STRING_ISO8601) });
		return ERROR_SCENARIO_BAD_TIMES;
	}
	if (!m_endTime.GetTime(0)) {
		if (v)
			/// <summary>
			/// The scenario end time is out of range or invalid.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTime", "endTime", validation::error_level::WARNING, validation::id::time_invalid, m_endTime.ToString(WTIME_FORMAT_STRING_ISO8601), { true, WTime::GlobalMin().ToString(WTIME_FORMAT_STRING_ISO8601) }, { true, WTime::GlobalMax().ToString(WTIME_FORMAT_STRING_ISO8601) });
		return ERROR_SCENARIO_BAD_TIMES;
	}
	if (m_endTime <= m_startTime) {
		if (v)
			/// <summary>
			/// The scenario start time occurs after the end time.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTime", { "startTime", "endTime" }, validation::error_level::WARNING, validation::id::time_invalid, { m_startTime.ToString(WTIME_FORMAT_STRING_ISO8601), m_endTime.ToString(WTIME_FORMAT_STRING_ISO8601) });
		return ERROR_SCENARIO_BAD_TIMES;
	}

	WTimeSpan ts = m_endTime - m_startTime;
	HRESULT hr;
	PolymorphicAttribute var;
	XY_Point ll, ur;
	double resolution;
	bool foundTimezone = false;
	double elev = -1.0;
	bool elevation;
	double FMC = -1.0;
	bool active;
    IgnitionNode<fireengine_float_type> *node;
	if (FAILED(hr = lockObjects()))													return hr;
	if (FAILED(hr = m_gridEngine->Valid(m_layerThread, m_startTime, ts, 0, nullptr))) {
		m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		if (v)
			/// <summary>
			/// One or more objects associated with the scenario are invalid for the simulation's duration.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("Simulation:Reset", "Valid", validation::error_level::SEVERE, validation::id::state_invalid, "", "One or more objects associated with the scenario are invalid for the simulation's duration.");
		return hr;
	}

	m_gridEngine->PreCalculationEvent(m_layerThread, m_startTime, 0, nullptr);
	m_gridEngine->PreCalculationEvent(m_layerThread, m_startTime, 1, nullptr);

	VectorEngineNode* ven = m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->PreCalculationEvent(m_startTime, 0, nullptr);
		ven->m_vectorEngine->PreCalculationEvent(m_startTime, 1, nullptr);
		ven = ven->LN_Succ();
	}
	AssetNode<fireengine_float_type>* an = m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->PreCalculationEvent(m_startTime, 0, nullptr);
		an->m_asset->PreCalculationEvent(m_startTime, 1, nullptr);
		an = an->LN_Succ();
	}

	if (FAILED(hr = checkWxGridObjects())) {
		if (v)
			/// <summary>
			/// One or more weather grid objects associated with the scenario are invalid for the simulation's duration.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("Simulation:Reset", "WxGridValid", validation::error_level::SEVERE, validation::id::state_invalid, "", "One or more weather grid objects associated with the scenario are invalid for the simulation's duration.");
		m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}

	if (FAILED(m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_XLLCORNER, &var))) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "xllcorner", validation::error_level::SEVERE,
				validation::id::missing_georeference, "");
		m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}
	try {
		ll.x = std::get<double>(var);
	}
	catch (std::bad_variant_access&) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "xllcorner", validation::error_level::SEVERE,
				validation::id::cannot_convert_type, "");
		weak_assert(false); m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}

	if (FAILED(m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_YLLCORNER, &var))) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "yllcorner", validation::error_level::SEVERE,
				validation::id::missing_georeference, "");
		m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}
	try {
		ll.y = std::get<double>(var);
	}
	catch (std::bad_variant_access&) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "yllcorner", validation::error_level::SEVERE,
				validation::id::cannot_convert_type, "");
		weak_assert(false); m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}

	if (FAILED(m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_XURCORNER, &var))) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "xurcorner", validation::error_level::SEVERE,
				validation::id::missing_georeference, "");
		m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}
	try {
		ur.x = std::get<double>(var);
	}
	catch (std::bad_variant_access&) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "xurcorner", validation::error_level::SEVERE,
				validation::id::cannot_convert_type, "");
		weak_assert(false); m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}

	if (FAILED(m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_YURCORNER, &var))) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "yurcorner", validation::error_level::SEVERE,
				validation::id::missing_georeference, "");
		m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}
	try {
		ur.y = std::get<double>(var);
	}
	catch (std::bad_variant_access&) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "yurcorner", validation::error_level::SEVERE,
				validation::id::cannot_convert_type, "");
		weak_assert(false); m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}

	if (FAILED(hr = m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_PLOTRESOLUTION, &var))) {
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "resolution", validation::error_level::SEVERE,
				validation::id::missing_georeference, "");
		m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}
	try {
		resolution = std::get<double>(var);
	}
	catch (std::bad_variant_access &) { 
		if (v)
			/// <summary>
			/// The plot resolution is not readable but should be by this time in scenario set-up.
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "resolution", validation::error_level::SEVERE,
				validation::id::cannot_convert_type, "");
		weak_assert(false); m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
		goto DONE;
	}

	node = m_impl->m_ignitionList.LH_Head();
	while (node->LN_Succ()) {							// retrieve all the data from the ignition COM objects - don't do any kind of processing on it, though
		if (m_impl->m_ignitionList.GetCount() == 1)
			node->Reset(m_ignitionOverrideTime, m_dt);
		else
			node->Reset(WTime((std::uint64_t)0, m_timeManager), m_dt);

		node = node->LN_Succ();
	}

	m_optionFlags &= (~(1ull << CWFGM_SCENARIO_OPTION_USE_2DGROWTH));

	if ((FAILED(m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_DEM_PRESENT, &var))) ||
	    ((FAILED(VariantToBoolean_(var, &elevation))) || (!elevation))) {
		weak_assert(std::holds_alternative<bool>(var));
		m_optionFlags &= (~(1ull << CWFGM_SCENARIO_OPTION_TOPOGRAPHY));
	}

	if (SUCCEEDED(m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_DEFAULT_FMC_ACTIVE, &var)) &&
		(SUCCEEDED(VariantToBoolean_(var, &active)))) {
		if (active) {
			if ((FAILED(m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_DEFAULT_FMC, &var))) ||
				(FAILED(VariantToDouble_(var, &FMC))) || (FMC < 0.0))
				FMC = -1.0;
		}
	}
	if ((FAILED(m_gridEngine->GetAttribute(m_layerThread, CWFGM_GRID_ATTRIBUTE_DEFAULT_ELEVATION, &var))) ||
	    (FAILED(VariantToDouble_(var, &elev))) || (elev < 0.0))
		elev = -1.0;
	if (elev < 0.0)
		elev = -1.0;

	if (m_impl->m_scenario) {
		weak_assert(false);
		delete m_impl->m_scenario;
	}

	try {
		m_impl->m_scenario = new Scenario<fireengine_float_type>(this, ll, ur, resolution, FMC, elev);
		hr = S_OK;
	}
	catch (std::exception& e) {
		if (v)
			/// <summary>
			/// Cannot allocate/initialize the object that encapsulates the simulation
			/// </summary>
			/// <type>internal</type>
			v->add_child_validation("Simulation:Reset", "Scenario", validation::error_level::SEVERE,
				validation::id::out_of_memory, "");
		hr = E_OUTOFMEMORY;
	}

    DONE:
	an = m_impl->m_scenario->m_scenario->m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->PostCalculationEvent(m_endTime, 1, nullptr);
		an->m_asset->PostCalculationEvent(m_endTime, 0, nullptr);
		an = an->LN_Succ();
	}
	ven = m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->PostCalculationEvent(m_endTime, 1, nullptr);
		ven->m_vectorEngine->PostCalculationEvent(m_endTime, 0, nullptr);
		ven = ven->LN_Succ();
	}

	m_gridEngine->PostCalculationEvent(m_layerThread, m_endTime, 1, nullptr);
	m_gridEngine->PostCalculationEvent(m_layerThread, m_endTime, 0, nullptr);

	return hr;
}


HRESULT CCWFGM_Scenario::Simulation_Step() {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_FALSE);
	if (!m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;
	m_impl->m_scenario->InitThreadPool((m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SINGLETHREADING)) ? false : true);
	HRESULT hr = m_impl->m_scenario->Step();
	weak_assert(hr == m_impl->m_scenario->m_stepState);
	return hr;
}


HRESULT CCWFGM_Scenario::Simulation_StepBack() {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_FALSE);
	if (!m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;
	if (!m_impl->m_scenario->m_timeSteps.GetCount())
		return ERROR_SCENARIO_BAD_STATE;
	return m_impl->m_scenario->StepBack();
}


HRESULT CCWFGM_Scenario::Simulation_Clear() {
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_TRUE);
	if (!m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	m_impl->unlockObjects(*this, nullptr, (AssetNode<fireengine_float_type>*)~0);
	delete m_impl->m_scenario;
	m_impl->m_scenario = nullptr;
	return S_OK;
}


HRESULT CCWFGM_Scenario::IsXYBurned(const XY_Point &pt, const HSS_Time::WTime &time, bool *burned) const {
	if (!burned)							return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime t(time, m_timeManager);
		XY_Point _pt(pt);
		m_impl->m_scenario->toInternal(_pt);
		return m_impl->m_scenario->PointBurned(_pt, &t, burned);
	}
	*burned = false;
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::GetBurnedBox(const HSS_Time::WTime &time, XY_Rectangle *box) const {
	if (!box)							return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		XY_RectangleTempl<fireengine_float_type> bbox;
		WTime t(time, m_timeManager);
		HRESULT hr = m_impl->m_scenario->GetBurningBox(&t, bbox);

		*box = bbox;
		m_impl->m_scenario->fromInternal(box->m_min);
		m_impl->m_scenario->fromInternal(box->m_max);
		return hr;
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::GetXYStats(const XY_Point &min_pt, const XY_Point &max_pt, XYStatOptions* options, std::uint16_t stat, NumericVariant *stats) {
	if (!options)										return E_POINTER;
	if (!stats)											return E_POINTER;
	if (!(options->time))								return ERROR_FIRE_INVALID_TIME;
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_FALSE);

	bool clear = false;
	HRESULT hr;
	if ((!m_impl->m_scenario) && (options->interp_method == SCENARIO_XYSTAT_TECHNIQUE_CALCULATE)) {
		_semaphore_engage.Unlock();
		if (FAILED(hr = Simulation_Reset(nullptr, "")))
			return hr;
		_semaphore_engage.Lock(false);
		clear = true;
	}

	if (m_impl->m_scenario) {
		WTime mt(options->mintime, m_timeManager);
		WTime t(options->time, m_timeManager);
		XY_Point _pt1(min_pt), _pt2(max_pt);
		m_impl->m_scenario->toInternal(_pt1);
		m_impl->m_scenario->toInternal(_pt2);
		hr = m_impl->m_scenario->GetStats(min_pt, max_pt, _pt1, _pt2, &mt, &t, 1, &stat, stats, false, options->interp_method, options->discretization, false);
		options->time = t.GetTime(0);
	} else	hr = ERROR_SCENARIO_BAD_STATE;

	if (clear) {
		_semaphore_engage.Unlock();
		Simulation_Clear();
	}
	return hr;
}


HRESULT CCWFGM_Scenario::GetXYStatsSet(const XY_Point& min_pt, const XY_Point& max_pt, XYStatOptions* options, std::vector<XYStat> *stats) {
	if (!options)									return E_POINTER;
	if (!stats)										return E_POINTER;
	if (!(options->time))							return ERROR_FIRE_INVALID_TIME;
	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_FALSE);

	bool clear = false;
	HRESULT hr;
	if ((!m_impl->m_scenario) && (options->interp_method == SCENARIO_XYSTAT_TECHNIQUE_CALCULATE)) {
		_semaphore_engage.Unlock();
		if (FAILED(hr = Simulation_Reset(nullptr, "")))
			return hr;
		_semaphore_engage.Lock(SEM_FALSE);
		clear = TRUE;
	}

	if (m_impl->m_scenario) {
		WTime mt(options->mintime, m_timeManager);
		WTime t(options->time, m_timeManager);
		ULONG cnt = stats->size();
		USHORT* stats_array = new USHORT[cnt];
		NumericVariant* stats_result = new NumericVariant[cnt];
		for (ULONG i = 0; i < cnt; i++)
			stats_array[i] = (*stats)[i].stat;
		XY_Point _pt1(min_pt), _pt2(max_pt);
		m_impl->m_scenario->toInternal(_pt1);
		m_impl->m_scenario->toInternal(_pt2);
		hr = m_impl->m_scenario->GetStats(min_pt, max_pt, _pt1, _pt2, &mt, &t, cnt, stats_array, stats_result, false, options->interp_method, options->discretization, false);
		if (SUCCEEDED(hr)) {
			for (ULONG i = 0; i < cnt; i++) {
				XYStat s;
				s.stat = (*stats)[i].stat;
				s.value = stats_result[i];
				(*stats)[i] = s;
			}
		}
		delete[] stats_array;
		delete[] stats_result;
		options->time = t.GetTime(0);
	}
	else
		hr = ERROR_SCENARIO_BAD_STATE;

	if (clear) {
		_semaphore_engage.Unlock();
		Simulation_Clear();
	}
	return hr;
}


HRESULT CCWFGM_Scenario::GetNumberSteps(std::uint32_t *steps) const {
	if (!steps)								return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario)
		return m_impl->m_scenario->GetNumSteps(steps);
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::GetStepsArray(std::uint32_t *size, std::vector<HSS_Time::WTime> *times) const {
	if (!size)									return E_POINTER;
	if (!times)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario)
	{
		return m_impl->m_scenario->GetStepsArray(size, times);
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::GetNumberFires( HSS_Time::WTime *time,  std::uint32_t *count) const {
	if (!time)									return E_POINTER;
	if (!count)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime t(*time, m_timeManager);
		HRESULT hr = m_impl->m_scenario->GetNumFires(count, &t);
		time->SetTime(t);
		return hr;
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::IgnitionAtFireIndex(std::uint32_t index,  HSS_Time::WTime *time, boost::intrusive_ptr<CCWFGM_Ignition> *fire) const {
	if (!time)									return E_POINTER;
	if (!fire)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime t(*time, m_timeManager);
		HRESULT hr = m_impl->m_scenario->GetIgnition(index, &t, fire);
		time->SetTime(t);
		return hr;
	}
	return S_OK;
}


HRESULT CCWFGM_Scenario::GetVectorSize(std::uint32_t fire, HSS_Time::WTime *time, std::uint32_t *size) const {
	if (!time)									return E_POINTER;
	if (!size)									return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime t(*time, m_timeManager);
		HRESULT hr = m_impl->m_scenario->GetVectorSize(fire, &t, size);
		time->SetTime(t);
		return hr;
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::GetVectorArray(std::uint32_t fire, HSS_Time::WTime *time, std::uint32_t *size, XY_Poly *xy_pairs) const {
	if (!time)									return E_POINTER;
	if (!size)									return E_POINTER;
	if (!xy_pairs)								return E_POINTER;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime t(*time, m_timeManager);
		HRESULT hr = m_impl->m_scenario->GetVectorArray(fire, &t, size, *xy_pairs);
		if (m_optionFlags & ((1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING) | (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN))) {
			auto pts = xy_pairs->Detach();
			for (std::uint32_t i = 0; i < (*size); i++)
				m_impl->m_scenario->fromInternal(pts[i]);
			xy_pairs->Attach(pts, *size);
		}
		time->SetTime(t);
		return hr;
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::GetStatsArray(std::uint32_t fire, HSS_Time::WTime *time, std::uint16_t stat, 
	std::uint32_t *size, std::vector<double> *stats) const {
	if (!time)									return E_POINTER;
	if (!size)									return E_POINTER;
	if (!stats)									return E_POINTER;
	if (!(time->GetTotalMicroSeconds()))		return ERROR_FIRE_INVALID_TIME;

	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime t(*time, m_timeManager);
		HRESULT hr = m_impl->m_scenario->GetStatsArray(fire, &t, stat, size, *stats);
		time->SetTime(t);
		return hr;
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::GetStats(std::uint32_t fire, ICWFGM_Fuel *fuel, HSS_Time::WTime *time,
	std::uint16_t stat, std::uint16_t discretization, PolymorphicAttribute *stats) const {
	if (!time)									return E_POINTER;
	if (!stats)									return E_POINTER;
	if (!(time->GetTotalMicroSeconds()))									return ERROR_FIRE_INVALID_TIME;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime t(*time, m_timeManager);
		HRESULT hr = m_impl->m_scenario->GetStats(fire, fuel, &t, stat, discretization, stats);
		time->SetTime(t);
		return hr;
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::GetStatsPercentage(const std::uint32_t fire,  HSS_Time::WTime *time, const std::uint16_t stat, const double greater_equal, const double less_than, double *stats) const {
	if (!time)									return E_POINTER;
	if (!stats)									return E_POINTER;
	if (!(time->GetTotalMicroSeconds()))		return ERROR_FIRE_INVALID_TIME;
	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime t(*time, m_timeManager);
		HRESULT hr = m_impl->m_scenario->GetStats(fire, &t, stat, true, greater_equal, less_than, stats);
		time->SetTime(t);
		return hr;
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::BuildCriticalPath(const ICWFGM_Asset* asset, const std::uint32_t index, const std::uint16_t flags,
	MinListTempl<CriticalPath>& polyset, const ScenarioExportRules* rules) const {

	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore &>(m_lock), SEM_FALSE);

	if (!m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	if ((m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_ASSET) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI90) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI95) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI100) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_RH) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_PRECIP) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_AREA) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_BURNDISTANCE))
		return ERROR_SCENARIO_BAD_STATE;

	AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
	while (node->LN_Succ()) {
		if ((node->m_asset == asset) || ((asset == nullptr) && (m_impl->m_assetList.GetCount() == 1))) {
			if (index != (std::uint32_t)-1) {
				AssetGeometryNode<fireengine_float_type>* g = node->m_geometry.IndexNode(index);
				if (!g)
					return ERROR_SCENARIO_ASSET_GEOMETRY_UNKNOWN;
				if (g->m_arrived) {
					CriticalPath* poly = new CriticalPath();
					HRESULT hr1 = m_impl->m_scenario->BuildCriticalPath(node, g, flags, poly, rules);
					if (SUCCEEDED(hr1)) {
						polyset.AddTail(poly);
					}
					else {
						delete poly;
					}
					return hr1;
				}
				return ERROR_SCENARIO_ASSET_NOT_ARRIVED;
			}
			else {
				AssetGeometryNode<fireengine_float_type>* g = node->m_geometry.LH_Head();
				HRESULT hr = ERROR_SCENARIO_ASSET_NOT_ARRIVED;

				while (g->LN_Succ()) {
					if (g->m_arrived) {
						CriticalPath* poly = new CriticalPath();
						HRESULT hr1 = m_impl->m_scenario->BuildCriticalPath(node, g, flags, poly, rules);
						if (SUCCEEDED(hr1)) {
							polyset.AddTail(poly);
							hr = S_OK;
						}
						else {
							delete poly;
						}
					}
					g = g->LN_Succ();
				}
				return hr;
			}
		}
		node = node->LN_Succ();
	}
	return ERROR_SCENARIO_ASSET_UNKNOWN;
}


HRESULT CCWFGM_Scenario::GetFuelData(const XY_Point& pt, const HSS_Time::WTime& time, boost::intrusive_ptr<ICWFGM_Fuel>* fuel, bool* fuel_valid, CCWFGM_FuelOverrides *overrides, XY_Rectangle* cache_bbox) {
	if (!fuel)								return E_POINTER;
	if (!fuel_valid)						return E_POINTER;
	if (!overrides)							return E_POINTER;

	CRWThreadSemaphoreEngage _semaphore_engage(m_lock, SEM_FALSE);

	HRESULT hr = ERROR_SCENARIO_BAD_STATE;
	bool clear = false;
	if (!m_impl->m_scenario) {
		_semaphore_engage.Unlock();
		if (FAILED(hr = Simulation_Reset(nullptr, "")))
			return hr;
		_semaphore_engage.Lock(false);
		clear = true;
	}

	if (m_impl->m_scenario) {
		*fuel = NULL;
		WTime t(time, m_timeManager);
		ICWFGM_Fuel *f = m_impl->m_scenario->GetFuelUTM_NotCached(t, pt, *fuel_valid);
		if ((*fuel_valid) && (f)) {
			m_impl->m_scenario->GetCorrectedFuelUTM(pt, t, f, *overrides);
		}
		hr = S_OK;
	}

	if (clear) {
		_semaphore_engage.Unlock();
		Simulation_Clear();
	}

	return hr;
}


HRESULT CCWFGM_Scenario::GetPercentileClassCount(unsigned char *count) {
	return m_sp.GetPercentileClassCount(count);
}


HRESULT CCWFGM_Scenario::GetPercentileFuel(unsigned char indexFuel, _GUID *defaultFuel) {
	return m_sp.GetPercentile(indexFuel, defaultFuel);
}


HRESULT CCWFGM_Scenario::SetPercentileValue(const _GUID *defaultFuel, unsigned char fireDescription, double s) {
	return m_sp.SetPercentileValue(defaultFuel, fireDescription, s);
}


HRESULT CCWFGM_Scenario::GetPercentileValue(const _GUID *defaultFuel, unsigned char fireDescription, double *s) {
	return m_sp.GetPercentileValue(defaultFuel, fireDescription, s);
}


HRESULT CCWFGM_Scenario::RSI(const _GUID *clsId, double RSIin, double CFBin,  double *RSIout) {
	if (!(m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE_ENABLE))) {
		*RSIout = RSIin;
		return S_OK;
	}

	double _tinv;
	if (!m_impl->m_scenario)
		_tinv = tinv(m_growthPercentile / 100.0, 9999999);
	else
		_tinv = m_impl->m_scenario->m_tinv;
	return m_sp.RSI(_tinv, clsId, RSIin, CFBin, RSIout);
}


double CCWFGM_Scenario::spatialThreshold(const double area) const {
	if (!(m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC)))
		return m_spatialThreshold;
	return (1.0 + 3.0 / (1.0 + exp(-1.6 * area / 10000.0 + 8.0)));
}


double CCWFGM_Scenario::perimeterResolution(const double area) const {
	if (!(m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC)))
		return m_perimeterResolution;
	return (1.0 + 3.0 / (1.0 + exp(-1.6 * area / 10000.0 + 8.0)));
}
 