/**
 * WISE_Scenario_Growth_Module: ScenarioTimeStep.cpp
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
#include "scenario.h"
#include "ScenarioTimeStep.h"
#include "ScenarioIgnition.h"
#include "results.h"
#include "GridCom_ext.h"
#include "FireEngine_ext.h"
#include "CWFGM_Scenario_Internal.h"
#include <omp.h>
#include <algorithm>

#ifdef ROB_5CM
#define EPSILON (0.05)
#else
#define EPSILON (1e-4)
#endif


IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(ScenarioFire, ScenarioFire, 1024 * 1024 / sizeof(ScenarioFire<fireengine_float_type>), false, 16, fireengine_float_type)
IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(ScenarioFireExport, ScenarioFireExport, 256 * 1024 / sizeof(ScenarioFireExport<fireengine_float_type>), false, 16, fireengine_float_type)
IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(ScenarioTimeStep, ScenarioTimeStep, 512 * 1024 / sizeof(ScenarioTimeStep<fireengine_float_type>), false, 16, fireengine_float_type)
IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(XY_PolyLLPolyRef, XY_PolyLLPolyRef, 1024 * 1024 / sizeof(XY_PolyLLPolyRef<fireengine_float_type>), false, 16, fireengine_float_type)


template<class _type>
ScenarioFire<_type>::ScenarioFire(const ScenarioTimeStep<_type> *timeStep, const IgnitionNode<_type> *ignition, ScenarioFire<_type> *pred) :
	m_timeStep(timeStep), m_ignition(ignition), m_calcPred(pred), m_calcSucc(nullptr) {
	if (m_calcPred)
		m_calcPred->setCalcSucc(this);
	m_fireArea = 0.0;
	m_initArea = 0.0;
	m_newVertexStatus = FP_FLAG_NORMAL;
	m_canBurn = 1;
	m_gusting = 0.0;

	if (m_timeStep)
		if (!(m_timeStep->m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
			SetCacheScale(m_timeStep->m_scenario->resolution());
}


template<class _type>
_type ScenarioFire<_type>::InitArea() const {
	if (m_initArea == 0.0)
		((ScenarioFire<_type>*)this)->m_initArea = Area();
	return m_initArea;
}


template<class _type>
ScenarioTimeStep<_type>::ScenarioTimeStep(Scenario<_type> *scenario, const WTime &event_end, bool simulation_end) : m_time(event_end) {
	m_lock.Lock_Write();

    #ifdef _DEBUG
	bool check_displayable = false, check_event = false;
    #endif

	m_displayable = 0;
	m_vectorBreaksLL = nullptr;
	m_scenario = scenario;
	m_evented = 0;
	m_ignitioned = 0;
	m_centroid.x = m_centroid.y = -99999999.0;

	m_scenario->m_timeSteps.AddTail(this);
	ScenarioTimeStep<_type> *sts = LN_Pred();

	XY_PointTempl<_type> centroid;
	WTime step_start((std::uint64_t)0, m_scenario->m_scenario->m_timeManager);
	if (sts->LN_Pred()) {
		step_start = sts->m_time;

		if (!sts->Centroid(&centroid))
			if (!findFirstIgnitionCentroid(centroid))
				centroid = m_scenario->start_center();
	}
	else {
		step_start = m_scenario->m_scenario->m_startTime;
		
//		go and get the centroid from ignitions, or just of the first ignition(s) - or maybe just of the entire clipped area since we have nothing to burn yet? - could affect the first time step for the first fire,
//		which could just be just 2 minutes if it's a point ignition

		if (!findFirstIgnitionCentroid(centroid))
			// get the centroid of the grid dataset
			centroid = m_scenario->start_center();
	}

#ifdef _DEBUG
	std::string etime = event_end.ToString(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST | WTIME_FORMAT_ABBREV | WTIME_FORMAT_DATE | WTIME_FORMAT_TIME);
	std::string stime = step_start.ToString(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST | WTIME_FORMAT_ABBREV | WTIME_FORMAT_DATE | WTIME_FORMAT_TIME);
#endif

	if (m_time == step_start) {
		PreCalculation();
		m_displayable = 1;
		return;								// we're the first time step so it's easy
	}

	m_time += WTimeSpan(0, 0, 0, 1);
	bool i_time_valid = false;
	WTime i_time(m_time);
	const WTime t_time(m_time);
	IgnitionNode<_type> *node = m_scenario->m_scenario->m_impl->m_ignitionList.LH_Head();
	while (node->LN_Succ()) {
		if ((node->m_ignitionTime > step_start) && (node->m_ignitionTime < m_time)) {
			i_time = m_time = node->m_ignitionTime;
			i_time_valid = true;
		}
		else {
			if ((m_scenario->m_scenario->m_sc.anythingValid()) && (m_scenario->m_scenario->m_sc.responseTime.GetTotalSeconds() > 0)) {
				WTime responseTime = node->m_ignitionTime + m_scenario->m_scenario->m_sc.responseTime;
				if ((responseTime > step_start) && (responseTime < m_time))
					m_time = responseTime;
			}
		}
		node = node->LN_Succ();
	}						// wind back the next time step

	WTime secs(m_time);

	PreCalculation();				// moved from scenario.cpp to here to lock GetEventTime() correctly

	bool secs_valid;
	m_scenario->m_scenario->m_gridEngine->GetEventTime(m_scenario->m_scenario->m_layerThread, centroid, CWFGM_GETEVENTTIME_FLAG_SEARCH_FORWARD, step_start, &secs, &secs_valid);
							// this takes care of the gridded data and the weather

	if (secs > m_time) {		// event time searching should be closer in time to step_start, no further away
		weak_assert(m_time >= secs);
		secs = m_time;
	}

	VectorEngineNode *ven = m_scenario->m_scenario->m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->GetEventTime(CWFGM_GETEVENTTIME_FLAG_SEARCH_FORWARD, step_start, &secs);
		ven = ven->LN_Succ();
	}						// this takes care of any vector data that may want to change (in the future)

	AssetNode<_type>* an = m_scenario->m_scenario->m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->GetEventTime(CWFGM_GETEVENTTIME_FLAG_SEARCH_FORWARD, step_start, &secs);
		an = an->LN_Succ();
	}						// this takes care of any vector data that may want to change (in the future)

	m_scenario->m_scenario->m_impl->m_go.GetEventTime(m_scenario, CWFGM_GETEVENTTIME_FLAG_SEARCH_FORWARD, step_start, &secs);

	m_time = secs;
	if (t_time != m_time) {
		m_evented = 1;
	} else {
		m_evented = 0;
		m_time = event_end;
	}
	// got everything we care about from the COM object, now let's check the burn condition
	WTimeSpan start_time_of_day = step_start.GetTimeOfDay(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	WTimeSpan time_of_day = m_time.GetTimeOfDay(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);

	m_evented |= (event_end != m_time);
	m_evented |= ((event_end == m_time) && simulation_end);
	ActiveFire<_type> *af = m_scenario->m_activeFires.LH_Head();
	while (af->LN_Succ()) {
		if (af->LN_Ptr()) {	// this can happen if the fire's area drops to 0 or is otherwise removed because of the untangler
			af->LN_Ptr()->CalculateEndTime();
			if (af->m_endTime < m_time) {
				m_evented = false;
				m_time = af->m_endTime;
			}
		}
		af = af->LN_Succ();
	}

	if ((m_time == event_end) || (!m_scenario->m_scenario->m_displayInterval.GetTotalSeconds()))
		m_displayable = 1;
	if ((i_time_valid) && (i_time == m_time))
		m_ignitioned = 1;

	m_tickCountStart = 0;
	m_tickCountEnd = 0;

	m_assetCount = 0;
}


template<class _type>
ScenarioTimeStep<_type>::~ScenarioTimeStep() {
	if (m_vectorBreaksLL) {
		std::uint32_t i, cnt = (std::uint32_t)m_vectorBreaksLL->size();
		for (i = 0; i < cnt; i++)
			delete m_vectorBreaksLL->at(i);
		delete m_vectorBreaksLL;
	}
	XY_PolyLLPolyRef<_type>* p;
	while (p = m_staticVectorBreaksLL.RemHead())
		delete p;
	ScenarioFire<_type> *sf;
	while (sf = m_fires.RemHead())
		delete sf;
	ActiveFire<_type>* af;
	while (af = m_activeFiresState.RemHead()) {
		af->Detach();
		delete af;
	}
}


template<class _type>
void ScenarioTimeStep<_type>::PreCalculation() {
	CalculationEventParms parms;

	ScenarioTimeStep<_type> *sts = LN_Pred();
	XYRectangleType bbox;
	bool set = false;
	if (sts->LN_Pred())
		set = sts->BoundingBox(bbox);
	if (!set)
		m_scenario->IgnitionExtents(bbox);

	parms.SimulationMin.x = bbox.m_min.x;
	parms.SimulationMin.y = bbox.m_min.y;
	parms.SimulationMax.x = bbox.m_max.x;
	parms.SimulationMax.y = bbox.m_max.y;

	m_scenario->m_scenario->m_gridEngine->PreCalculationEvent(m_scenario->m_scenario->m_layerThread, m_time, 1, &parms);

	m_curr_ll.x = parms.CurrentGridMin.x;
	m_curr_ll.y = parms.CurrentGridMin.y;
	m_curr_ur.x = parms.CurrentGridMax.x;
	m_curr_ur.y = parms.CurrentGridMax.y;

	VectorEngineNode *ven = m_scenario->m_scenario->m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->PreCalculationEvent(m_time, 1, &parms);
		ven = ven->LN_Succ();
	}
	AssetNode<_type>* an = m_scenario->m_scenario->m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->PreCalculationEvent(m_time, 1, &parms);
		an = an->LN_Succ();
	}

	XYPointType gridmin, gridmax;
	gridmin.x = parms.TargetGridMin.x;
	gridmin.y = parms.TargetGridMin.y;
	gridmax.x = parms.TargetGridMax.x;
	gridmax.y = parms.TargetGridMax.y;
	m_scenario->Size(gridmin, gridmax);
}


template<class _type>
void ScenarioTimeStep<_type>::PostCalculation() {
	CalculationEventParms parms;

	AssetNode<_type>* an = m_scenario->m_scenario->m_impl->m_assetList.LH_Head();
	while (an->LN_Succ()) {
		an->m_asset->PostCalculationEvent(m_time, 1, &parms);
		an = an->LN_Succ();
	}

	VectorEngineNode* ven = m_scenario->m_scenario->m_vectorEngineList.LH_Head();
	while (ven->LN_Succ()) {
		ven->m_vectorEngine->PostCalculationEvent(m_time, 1, &parms);
		ven = ven->LN_Succ();
	}

	XYRectangleType bbox;
	bool set = BoundingBox(bbox);
	if (!set)
		m_scenario->IgnitionExtents(bbox);

	parms.SimulationMin.x = bbox.m_min.x;
	parms.SimulationMin.y = bbox.m_min.y;
	parms.SimulationMax.x = bbox.m_max.x;
	parms.SimulationMax.y = bbox.m_max.y;

	m_scenario->m_scenario->m_gridEngine->PostCalculationEvent(m_scenario->m_scenario->m_layerThread, m_time, 1, &parms);

	XYPointType gridmin, gridmax;
	gridmin.x = parms.TargetGridMin.x;
	gridmin.y = parms.TargetGridMin.y;
	gridmax.x = parms.TargetGridMax.x;
	gridmax.y = parms.TargetGridMax.y;
	m_scenario->Size(gridmin, gridmax);
}


template<class _type>
const typename ScenarioTimeStep<_type>::XYPointType& ScenarioTimeStep<_type>::current_ll() const {
	return m_curr_ll;
}


template<class _type>
const typename ScenarioTimeStep<_type>::XYPointType& ScenarioTimeStep<_type>::current_ur() const {
	return m_curr_ur;
}


template<class _type>
std::uint64_t ScenarioTimeStep<_type>::toGridScaleX(const XY_Point& loc) const {
	double x = loc.x;
	x -= m_curr_ll.x;
	x *= m_scenario->iresolution();
	return (std::uint64_t)x;
}


template<class _type>
std::uint64_t ScenarioTimeStep<_type>::toGridScaleY(const XY_Point& loc) const {
	double y = loc.y;
	y -= m_curr_ll.y;
	y *= m_scenario->iresolution();
	return (std::uint64_t)y;
}

#ifdef USE_BIGFLOATS
template<class _type>
std::uint64_t ScenarioTimeStep<_type>::toGridScaleX(const XYPointType& loc) const {
	_type x = loc.x;
	x -= m_curr_ll.x;
	x *= m_scenario->iresolution();
	return (std::uint64_t)x;
}


template<class _type>
std::uint64_t ScenarioTimeStep<_type>::toGridScaleY(const XYPointType& loc) const {
	_type y = loc.y;
	y -= m_curr_ll.y;
	y *= m_scenario->iresolution();
	return (std::uint64_t)y;
}
#endif

template<class _type>
FireFront<_type>*ScenarioFire<_type>::New() const {
	FireFront<_type> *ff = new FireFront<_type>(this);
	ff->SetCacheScale(GetCacheScale());
	return ff;
}


template<class _type>
FireFront<_type>*ScenarioFire<_type>::NewCopy(const XYPolyLLType &toCopy) const {
	FireFront<_type> *ff = new FireFront(this, (const FireFront<_type>&)toCopy);
	ff->SetCacheScale(GetCacheScale());
	return ff;
}


template<class _type>
FireFrontExport<_type>*ScenarioFireExport<_type>::New() const {
	FireFrontExport<_type> *ff = new FireFrontExport<_type>(this);
	ff->SetCacheScale(GetCacheScale());
	return ff;
}


template<class _type>
FireFrontExport<_type> *ScenarioFireExport<_type>::NewCopy(const XYPolyLLType &toCopy) const {
	FireFrontExport<_type> *ff = new FireFrontExport(this, (const FireFront<_type>&)toCopy);
	ff->SetCacheScale(GetCacheScale());
	return ff;
}


template<class _type>
double ScenarioTimeStep<_type>::MinimumROSRatio() const {
	double min_ratio = 10.0;
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		double ratio = sf->MinimumROSRatio();
		if (ratio < min_ratio)
			min_ratio = ratio;
		sf = sf->LN_Succ();
	}
	return min_ratio;
}


template<class _type>
double ScenarioTimeStep<_type>::MaximumROS() const {
	double max_ros = 0.0;
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		double m_ros = sf->MaximumROS();
		if (m_ros > max_ros)
			max_ros = m_ros;
		sf = sf->LN_Succ();
	}
	return max_ros;
}


template<class _type>
double ScenarioTimeStep<_type>::MaximumCardinalROS() const {
	double max_ros = 0.0;
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		double m_ros = sf->MaximumCardinalROS();
		if (m_ros > max_ros)
			max_ros = m_ros;
		sf = sf->LN_Succ();
	}
	return max_ros;
}


template<class _type>
std::uint32_t ScenarioTimeStep<_type>::GetNumFireFronts() const {
	std::uint32_t fires = 0;
	ScenarioFire<_type>*sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		fires += sf->NumPolys();
		sf = sf->LN_Succ();
	}
	return fires;
}


template<class _type>
bool ScenarioTimeStep<_type>::Centroid(XY_PointTempl<_type>* centroid) {

	CThreadSemaphoreEngage engage(&m_c_lock, SEM_TRUE);
	if ((m_centroid.x != -99999999.0) && (m_centroid.y != -99999999.0)) {
		if (m_fires.GetCount()) {
			*centroid = m_centroid;
			return true;
		}
		return false;
	}
	double cnt = 0.0;
	centroid->x = CONSTANTS_NAMESPACE::C<_type>(0.0);
	centroid->y = CONSTANTS_NAMESPACE::C<_type>(0.0);
	if (m_fires.GetCount()) {
		ScenarioFire<_type>* sf = m_fires.LH_Head();
		while (sf->LN_Succ()) {
			XY_PointTempl<_type> c;
			sf->Centroid(c, true);
			(*centroid) += c;
			cnt++;
			sf = sf->LN_Succ();
		}
		(*centroid) /= cnt;
		m_scenario->fromInternal(*centroid);
		m_centroid = *centroid;
		return true;
	}
	return false;
}


template<class _type>
const FireFront<_type> *ScenarioTimeStep<_type>::GetFireFront(std::uint32_t index) {
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		if (index >= sf->NumPolys())
			index -= sf->NumPolys();
		else
			return sf->GetPoly(index);
		sf = sf->LN_Succ();
	}
	return NULL;
}


template<class _type>
bool ScenarioTimeStep<_type>::BoundingBox(XYRectangleType &bbox) const {
	if (!m_fires.GetCount()) {
		bbox.m_min.y = bbox.m_min.x = DBL_MAX;
		bbox.m_max.y = bbox.m_max.x = -DBL_MAX;
		return false;
	}
	ScenarioFire<_type> *fs = m_fires.LH_Head();
	bool one = fs->BoundingBox(bbox);
	fs = fs->LN_Succ();
	while (fs->LN_Succ()) {
		XYRectangleType b;
		if (fs->NumPolys()) {
			if (fs->BoundingBox(b)) {
				if (!one) {
					bbox = b;
					one = true;
				} else
					bbox.EncompassRectangle(b);
			}
		}
		fs = fs->LN_Succ();
	}
	return one;
}


template<class _type>
std::int32_t ScenarioTimeStep<_type>::PointInArea(const XYPointType &pt) const {
	std::int32_t val = 0;
	std::int32_t on_hull = 0;
	std::int32_t retval;
	ScenarioFire<_type> *fs = m_fires.LH_Head();
	while (fs->LN_Succ()) {
		if (fs->FastCollisionTest(pt, 0.0)) {
			retval = fs->PointInArea(pt, 0.0/*, pool*/);
			val += (retval & (~1));
			on_hull |= (retval & 1);
		}
		fs = fs->LN_Succ();
	}
	return val + on_hull;
}


template<class _type>
FirePoint<_type> *ScenarioTimeStep<_type>::GetNearestPoint(const XYPointType &pt, bool all_points, FireFront<_type> **firefront, bool must_be_inside) const {
	FirePoint<_type> *fp = NULL;
	ScenarioFire<_type> *fs = m_fires.LH_Head();
	_type d2;
	if (firefront)
		*firefront = nullptr;

#ifdef _DEBUG
	LONG in = PointInArea(pt);
	if (must_be_inside)
		weak_assert(in);		// would have already passed this test
#endif

	while (fs->LN_Succ()) {
		FireFront<_type> *ff = fs->LH_Head();
		while (ff->LN_Succ()) {
			if ((!must_be_inside) || ff->PointInArea(pt)) {
				FirePoint<_type>* new_fp = ff->GetNearestPoint(pt, all_points);
				if (!fp) {
					fp = new_fp;
					if (fp) {
						d2 = fp->DistanceToSquared(pt);
						if (firefront)
							*firefront = ff;
					}
				} else if (new_fp) {
					_type new_d2 = new_fp->DistanceToSquared(pt);
					weak_assert(d2 == fp->DistanceToSquared(pt));
					if (d2 > new_d2) {
						fp = new_fp;
						d2 = new_d2;
						if (firefront)
							*firefront = ff;
					}
				}
			}
			ff = ff->LN_Succ();
		}
		fs = fs->LN_Succ();
	}
	return fp;
}


template<class _type>
void ScenarioTimeStep<_type>::advanceFire(ActiveFire<_type> *af) {
	ScenarioFire<_type> *sf = af->LN_Ptr();
	ScenarioFire<_type> *sf_new = new ScenarioFire<_type>(this, sf->Ignition(), sf);
	sf_new->m_activeFire = sf->m_activeFire;
	if ((m_evented && (!m_ignitioned)) ||
		(af->m_endTime == m_time) ||
		((!m_displayable) && (!(m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS))))) {
		weak_assert(sf_new->m_activeFire->LN_Ptr() == sf);
		sf_new->m_activeFire->LN_Ptr(sf_new);	// only update the pointer to the current image of this fire if we can actually calculate from it
		sf_new->BoundingBox(sf_new->m_activeFire->m_boundingBox);
		XYRectangleType bbox(sf->m_activeFire->m_boundingBox);
	}

#ifdef DEBUG
	weak_assert(sf->NumPolys());
#endif

	FireFront<_type> *ff = sf->LH_Head();
	while (ff->LN_Succ()) {						// this copies the fires from the previous state to this one, to advance them here
		weak_assert(ff->Fire() == sf);
		weak_assert(ff->Fire() != sf_new);
		if (ff->NumPoints() >= 3) {
			FireFront<_type> *new_ff = new FireFront<_type>(sf_new, *ff);
			if (!(m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
				new_ff->SetCacheScale(sf_new->GetCacheScale());
			sf_new->AddPoly(new_ff);
		}
		ff = ff->LN_Succ();
	}
	m_fires.AddTail(sf_new);
	sf_new->InitArea(sf->Area());
}


template<class _type>
bool ScenarioTimeStep<_type>::AdvanceFires() {					// copies fires from the previous step, "grows" them based on the length of
														// the time step from prev to this, and smooths automatically
	ScenarioTimeStep<_type> *pred = LN_Pred();
	if (!pred->LN_Pred())
		return false;									// no fires in the previous time step (no previous time step) so this is easy

#ifdef _DEBUG
	std::string mtime = m_time.ToString(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST | WTIME_FORMAT_ABBREV | WTIME_FORMAT_DATE | WTIME_FORMAT_TIME);
#endif

	std::uint32_t af_cnt = 0;
	ActiveFire<_type> *af = m_scenario->m_activeFires.LH_Head();
	while (af->LN_Succ()) {
		if (af->LN_Ptr()) {
			if (m_evented ||								// gotta re-sync all fires if we've encountered an event interrupt
				(af->m_endTime == m_time) ||				// gotta add a fire if our timestep matches the fire's next calc step
				(!(m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS))) ||
				// gotta add it if fires are not independent
				(m_displayable && (m_scenario->m_scenario->m_displayInterval.GetTotalSeconds()))) {
				// it can be displayable if all timesteps are, or if it's the end of a Simulation_Step() call,
				// so don't necessarily add it if we want all timesteps
				advanceFire(af);
				af->m_advanced = 1;
				af_cnt++;
			}
			else
				af->m_advanced = 0;
		}
		else
			af->m_advanced = 0;
		af = af->LN_Succ();
}

	if ((m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS)) && (af_cnt != m_scenario->m_activeFires.GetCount())) {
		af = m_scenario->m_activeFires.LH_Head();		// if we've already determined they are close enough to each other to link them, then trivially activate them
		while (af->LN_Succ()) {
			if ((af->LN_Ptr()) && (!af->m_advanced)) {
				ActiveFire<_type> *f = af->Mate();
				while (f != af) {
					if (f->m_advanced) {
						af->m_endTime = m_time;
						advanceFire(af);
						af->m_advanced = 1;
						af_cnt++;
						break;
					}
					f = f->Mate();
					}
				}
			af = af->LN_Succ();
			}
		}

	if ((m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS)) && (af_cnt != m_scenario->m_activeFires.GetCount())) {
		_type area = 0.0;
		ScenarioFire<_type> *sf = pred->m_fires.LH_Head();
		while (sf->LN_Succ()) {
			_type a(sf->Area());
			if (a > area)
				area = a;
			sf = sf->LN_Succ();
		}
		m_scenario->fromInternal2D(area);
		area = area / 10000.0;//Ha
		double st = m_scenario->m_scenario->spatialThreshold(area);
		m_scenario->gridToInternal1D(st);

	REPEAT_ADD:
		if (af_cnt != m_scenario->m_activeFires.GetCount()) {
			af = m_scenario->m_activeFires.LH_Head();
			while (af->LN_Succ()) {
				if ((af->LN_Ptr()) && (!af->m_advanced)) {
					ActiveFire<_type> *f = m_scenario->m_activeFires.LH_Head();
					while (f->LN_Succ()) {
						if (f->m_advanced) {
							ScenarioFire<_type> *sf = af->LN_Ptr();
							if (sf->FastCollisionTest(*f->LN_Ptr(), st)) {
								FireFront<_type> *ff = sf->LH_Head(),
									*ff1 = f->LN_Ptr()->LH_Head();
								while (ff->LN_Succ()) {
									while (ff1->LN_Succ()) {
										if (ff->FastCollisionTest(*ff1, st)) {
											FirePoint<_type> *fp = ff->LH_Head();
											while (fp->LN_Succ()) {
												if (ff1->WithinDistance(*fp, st * 2.0, false)) {
													af->m_endTime = m_time;
													advanceFire(af);
													weak_assert(!af->Attached(f));
													af->Attach(f);
													af->m_advanced = 1;
													af_cnt++;
													goto REPEAT_ADD;
												}
												fp = fp->LN_Succ();
											}
										}
										ff1 = ff1->LN_Succ();
									}
									ff = ff->LN_Succ();
								}
							}
						}
						f = f->LN_Succ();
					}
				}
				af = af->LN_Succ();
	        }
	    }
	}

	bool advanced = false;
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		WTimeSpan elapsed;
        if (sf->LN_CalcPred()) {
            if (m_time == sf->LN_CalcPred()->TimeStep()->m_time) {
                weak_assert(false);	// what does this mean?
            }
            elapsed = m_time - sf->LN_CalcPred()->TimeStep()->m_time;
        } else {
			weak_assert(false);
			elapsed = m_time - pred->m_time;
		}

		_type scale = ((_type)elapsed.GetTotalSeconds()) / 60.0;
		m_scenario->toInternal1D(scale);
		// to adjust from m/min to grid/sec - ROS is returned in m/min and we need to
		// adjust that to "cell-size" (1/resolution)/sec to go from ROS to our
		// ellipse A, B, C
		sf->m_gusting = m_scenario->m_scenario->m_impl->m_go.AssignPercentGusting(sf, m_time);
		advanced |= sf->AdvanceFire(scale);
		sf = sf->LN_Succ();
	}

	return advanced;
}


template<class _type>
bool ScenarioFire<_type>::AdvanceFire(const _type scale) {
	bool advanced = false;
	FireFront<_type> *ff = LH_Head();
	while (ff->LN_Succ()) {
		advanced |= ff->AdvanceFire(scale);
		ff = ff->LN_Succ();
	}
	return advanced;
}


template<class _type>
void ScenarioTimeStep<_type>::SimplifyFires() {
	ScenarioFire<_type> *sf;

	sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		FireFront<_type> *ff = sf->LH_Head();
		while (ff->LN_Succ()) {
			ff->Simplify();
			ff = ff->LN_Succ();
		}
		sf = sf->LN_Succ();
	}
}


template<class _type>
void ScenarioTimeStep<_type>::SimplifyFiresNull() {

#ifdef _DEBUG
	ScenarioFire<_type> *sf;
	sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		FireFront<_type> *ff = sf->LH_Head();
		while (ff->LN_Succ()) {
			FirePoint<_type> *fp = ff->LH_Head();
			while (fp->LN_Succ()) {
				fp = fp->LN_Succ();
			}
			ff = ff->LN_Succ();
		}
		sf = sf->LN_Succ();
	}
#endif

}


template<class _type>
void ScenarioTimeStep<_type>::TrackFiresNull() {
	ScenarioFire<_type> *ff;
	ff = m_fires.LH_Head();
	while (ff->LN_Succ()) {
		FireFront<_type> *ff1 = ff->LH_Head();
		while (ff1->LN_Succ()) {
			ff1->EnableCaching(true);
			ff1 = ff1->LN_Succ();
		}
		ff->RescanRanges(false, m_scenario->m_multithread);
		ff = ff->LN_Succ();
	}

	ScenarioTimeStep<_type> *sts = LN_Pred();
	if (sts->LN_Pred()) {
		ff = sts->m_fires.LH_Head();
		while (ff->LN_Succ()) {
			ff->RescanRanges(false, m_scenario->m_multithread);
			ff = ff->LN_Succ();
		}
	}

}


template<class _type>
void ScenarioTimeStep<_type>::buildVectorBreaks() {
	const WTime zero((uint64_t)0, m_scenario->m_scenario->m_timeManager);
	if (!m_vectorBreaksLL) {
		m_vectorBreaksLL = new std::vector<XY_PolyLLSetBB<_type>*>;

		VectorEngineNode *ven = m_scenario->m_scenario->m_vectorEngineList.LH_Head();
		while (ven->LN_Succ()) {
			std::uint32_t size, num, br_size, i;
			if (SUCCEEDED(ven->m_vectorEngine->GetFireBreakCount(m_time, &num)) && (num)) {
				if (SUCCEEDED(ven->m_vectorEngine->GetFireBreakSize((std::uint32_t)-1, (std::uint32_t)-1, m_time, &size)) && (size)) {
					XY_Poly poly(size, XY_Poly::ALLOCSTYLE_NEWDELETE);
					bool first = true;

					for (i = 0; i < num; i++) {
						std::uint32_t i_i, i_num;
						if (SUCCEEDED(ven->m_vectorEngine->GetFireBreakSetCount(m_time, i, &i_num))) {
							XY_PolyLLSetBB<_type> *poly_set = new XY_PolyLLSetBB<_type>();
							for (i_i = 0; i_i < i_num; i_i++) {
								br_size = size;

								HRESULT hr = ven->m_vectorEngine->GetFireBreak(i, i_i, m_time, &br_size, &poly);

								if (SUCCEEDED(hr) && (br_size)) {
									XY_PolyLLTimed<_type>* poly_ll = poly_set->New();
									if (!(m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
										poly_ll->SetCacheScale(m_scenario->resolution());
									for (std::uint32_t iii = 0; iii < poly.NumPoints(); iii++) {
										XY_PolyLLNode<_type>* n = poly_ll->New();
										auto p = poly.GetPoint(iii);
										n->x = p.x;
										n->y = p.y;
										poly_ll->AddPoint(n);
									}

									poly_ll->m_publicFlags = XY_PolyLLTimed<_type>::Flags::INTERPRET_POLYGON;
									poly_ll->CleanPoly(0.0, XY_PolyLLTimed<_type>::Flags::INTERPRET_POLYGON);

									SHORT rotation = poly_ll->DetermineRotation();
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
								}
							}
							poly_set->RescanRanges(false, m_scenario->m_multithread);
							m_vectorBreaksLL->push_back(poly_set);
						}
					}
				}
			}
			ven = ven->LN_Succ();
		}
	}
}


template<class _type>
void ScenarioTimeStep<_type>::reviewStaticBreaks() {				// record the first time that any fire may come in contact with static vector break
	std::uint32_t i, i_cnt = m_scenario->StaticVectorBreakCount();

	ScenarioFire<_type>* sf;
	for (i = 0; i < i_cnt; i++) {
		const XY_PolyLLSetBB<_type>* vb = m_scenario->StaticVectorBreak(i);

		sf = m_fires.LH_Head();
		while (sf->LN_Succ()) {
			if (sf->FastCollisionTest(vb->box, 0.0)) {
				XY_PolyLLTimed<_type>* p = vb->LH_Head();
				while (p->LN_Succ()) {
					if (!p->m_usedTime.GetTime(0)) {
						if (sf->FastCollisionTest(*p, 0.0)) {
							p->m_usedTime = m_time;

							XY_PolyLLPolyRef<_type>* r = new XY_PolyLLPolyRef<_type>();
							r->LN_Ptr(p);
							m_staticVectorBreaksLL.AddTail(r);
						}
					} else {
						XY_PolyLLPolyRef<_type>* r = new XY_PolyLLPolyRef<_type>();
						r->LN_Ptr(p);
						m_staticVectorBreaksLL.AddTail(r);
					}
					p = p->LN_Succ();
				}
			}
			sf = sf->LN_Succ();
		}
	}
}


template<class _type>
void ScenarioTimeStep<_type>::TrackFires() {						// ray traces looking for barriers
	ScenarioFire<_type> *ff;
	const WTime zero((uint64_t)0, m_scenario->m_scenario->m_timeManager);

	buildVectorBreaks();
	reviewStaticBreaks();

	ff = m_fires.LH_Head();
	while (ff->LN_Succ()) {
		FireFront<_type> *ff1 = ff->LH_Head();
		while (ff1->LN_Succ()) {
			ff1->TrackFireGrid();
			ff1 = ff1->LN_Succ();
		}
		ff = ff->LN_Succ();
	}

	ff = m_fires.LH_Head();
	while (ff->LN_Succ()) {
		FireFront<_type> *ff1 = ff->LH_Head();
		while (ff1->LN_Succ()) {
			ff1->EnableCaching(true);
			ff1 = ff1->LN_Succ();
		}
		ff->RescanRanges(false, m_scenario->m_multithread);
		ff = ff->LN_Succ();
	}

	ScenarioTimeStep<_type> *sts = LN_Pred();
	if (sts) {
		ff = sts->m_fires.LH_Head();
		while (ff->LN_Succ()) {
			ff->RescanRanges(false, m_scenario->m_multithread);
			ff = ff->LN_Succ();
		}
	}

	ff = m_fires.LH_Head();
	while (ff->LN_Succ()) {
		FireFront<_type> *ff1 = ff->LH_Head();
		while (ff1->LN_Succ()) {
			ff1->TrackFireVector();
			ff1 = ff1->LN_Succ();
		}
		ff = ff->LN_Succ();
	}
}


template<class _type>
void ScenarioFire<_type>::inspectPolygons(const PolysetOperation /*operation*/) {
	FireFront<_type> *ff = LH_Head(), *next;
	bool remove;
	while (ff->LN_Succ()) {
		remove = false;
		if (ff->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERIOR_SPECIFIED) {
			if (ff->NumPoints() < 3)
				remove = true;
			else {
				FirePoint<_type> *fp = ff->LH_Head();
				while (fp->LN_Succ()) {			// search for an intersection - any intersection
					if ((fp->m_intersectionInfo) && (fp->m_intersectionInfo != (IntersectionType*)(~0)))
						break;
					fp = fp->LN_Succ();
				}
				if (!fp->LN_Succ()) {			// we hit the break
					std::int16_t orientation = ff->DetermineRotation();
					if (orientation < 0)
						remove = true;
				}
			}
		}
		if (remove) {
			next = ff->LN_Succ();
			RemovePoly(ff);
			delete ff;
			ff = next;
		} else	ff = ff->LN_Succ();
	}
}


template<class _type>
bool ScenarioFire<_type>::keepPolygon(XY_PolyLL_BaseTempl<_type>*poly, bool suggest_keeping, const PolysetOperation operation) const {
	_type min_area = TimeStep()->m_scenario->minFireArea();
	if (poly->NumPoints() < 3) {
		FireFront<_type>* ff = (FireFront<_type>*)poly;
		FirePoint<_type>* fp = ff->LH_Head();
		bool successful_breach = false;
		while (fp->LN_Succ()) {
			if (fp->m_successful_breach) {
				successful_breach = true;
				fp->m_successful_breach = false;
				break;
			}
			fp = fp->LN_Succ();
		}
		return false;
	}
	if (!suggest_keeping) {
		return suggest_keeping;
	}
	FireFront<_type> *ff = (FireFront<_type>*)poly;
	FirePoint<_type> *fp = ff->LH_Head();
	bool active = false;
	bool successful_breach = false;
	while (fp->LN_Succ()) {
		if (!fp->m_status) {
			active = true;
			if (successful_breach)
				break;
		}
		if (fp->m_successful_breach) {
			successful_breach = true;
			if (active)
				break;
		}
		fp = fp->LN_Succ();
	}
	if (poly->m_publicFlags & XY_PolyLL_BaseTempl<_type>::Flags::INTERIOR_SPECIFIED) {
		if (active) {
			_type area = poly->Area();
			if ((area < min_area) && (!successful_breach))
				return false;
		}
	} else {
		if (!active) {
			_type area = poly->Area();
			if ((area < min_area))			// if nothing is active, then nothing can breach
				return false;
		}
	}
	return true;
}


template<class _type>
void ScenarioTimeStep<_type>::UnWindFires(bool advance) {				// removes knots from existing fires
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		if (advance)
			sf->Unwind(false, m_scenario->m_multithread, &m_advanceMetrics, nullptr);

		sf->m_fireArea = sf->Area();
		sf = sf->LN_Succ();
	}
}


template<class _type>
void ScenarioTimeStep<_type>::UnOverlapFiresNull() {
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		sf->m_fireArea = sf->Area();
		sf = sf->LN_Succ();
	}
}


template<class _type>
void ScenarioTimeStep<_type>::UnOverlapFires() {					// ***** this still has to be done - this is essentially performing polygon subtraction - when we do this, we
										// also have to test for fires completely within another, and we can remove that part of the code in
										// trackPoints()
										// ***** really, this method should be subtracting one fire front from another (or vector break), but for now,
										// this should do a reasonable approximation mock-up
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	std::uint32_t i, i_cnt;
	while (sf->LN_Succ()) {
		FireFront<_type> *ff = sf->LH_Head();
		while (ff->LN_Succ()) {
			FirePoint<_type> *fp = ff->LH_Head();
			while (fp->LN_Succ()) {
				if (!fp->m_status) {
					auto p = m_staticVectorBreaksLL.LH_Head();
					while (p->LN_Succ()) {
						const XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>* v = p->LN_Ptr();
						if (v->PointInArea(*fp, 0.0)) {
							fp->m_status = FP_FLAG_VECTOR;
							break;
						}
						p = p->LN_Succ();
					}
					if (!fp->m_status) {
						if (m_vectorBreaksLL)
							i_cnt = (std::uint32_t)m_vectorBreaksLL->size();
						else
							i_cnt = 0;
						for (i = 0; i < i_cnt; i++) {
							if ((*m_vectorBreaksLL)[i]->box.PointInside(*fp))
								if ((*m_vectorBreaksLL)[i]->PointInArea(*fp, 0.0))
									fp->m_status = FP_FLAG_VECTOR;
						}
					}
				}
				if (!fp->m_status) {
					ScenarioFire<_type> *test = m_fires.LH_Head();
					while (test->LN_Succ()) {
						if (test != sf) {
							if (test->PointInArea(*fp, 0.0))
								fp->m_status = FP_FLAG_FIRE;
						}
						test = test->LN_Succ();
					}
				}
				fp = fp->LN_Succ();
			}
			ff = ff->LN_Succ();
		}
		sf = sf->LN_Succ();
	}

	sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		sf->m_newVertexStatus = FP_FLAG_FIRE;
		ScenarioFire<_type> *sf2 = m_fires.LH_Head();
		while (sf2->LN_Succ()) {
			if ((sf2 != sf) && (sf2->m_fireArea >= sf->m_fireArea))
				if (sf->FastCollisionTest(*sf2, 0.0)) {
					FireFront<_type> *ff = sf2->LH_Head();
					while (ff->LN_Succ()) {
						ff->CleanPoly(0.0, FireFront<_type>::Flags::INTERPRET_POLYGON);
						ff = ff->LN_Succ();
					}
					ff = sf->LH_Head();
					while (ff->LN_Succ()) {
						ff->CleanPoly(0.0, FireFront<_type>::Flags::INTERPRET_POLYGON);
						ff = ff->LN_Succ();
					}

					sf->ClipAgainst(*sf2, PolysetOperation::DIFF, m_scenario->m_multithread, &m_setMetrics, nullptr);
				}
			sf2 = sf2->LN_Succ();
		}
		sf->m_newVertexStatus = FP_FLAG_VECTOR;

		if (m_vectorBreaksLL)
			i_cnt = (std::uint32_t)m_vectorBreaksLL->size();
		else	i_cnt = 0;
		for (i = 0; i < i_cnt; i++)
			if (sf->FastCollisionTest(*((*m_vectorBreaksLL)[i]), 0.0)) {
				sf->ClipAgainst(*((*m_vectorBreaksLL)[i]), PolysetOperation::DIFF, m_scenario->m_multithread, &m_setMetrics, nullptr);
			}

		i_cnt = m_scenario->StaticVectorBreakCount();
		for (i = 0; i < i_cnt; i++) {
			XY_PolyLLSetBB<_type> *it = (XY_PolyLLSetBB<_type>*)m_scenario->StaticVectorBreak(i);
			if (sf->FastCollisionTest(*it, 0.0)) {
				bool relevant = false;
				auto p = it->LH_Head();
				while (p->LN_Succ()) {
					if (p->Participates(m_time)) {
						relevant = true;
						break;
					}
					p = p->LN_Succ();
				}
				if (relevant)
					sf->ClipAgainst(*it, PolysetOperation::DIFF, m_scenario->m_multithread, &m_setMetrics, &m_time);
			}
		}

		sf->m_fireArea = sf->Area();

		sf->m_newVertexStatus = FP_FLAG_NORMAL;
		sf = sf->LN_Succ();
	}
}


template<class _type>
bool ScenarioTimeStep<_type>::AddIgnitions() {						// adds newly started ignitions
	bool added = false;
	IgnitionNode<_type> *ignition = m_scenario->m_scenario->m_impl->m_ignitionList.LH_Head();

	WTime prev((std::uint64_t)0, m_scenario->m_scenario->m_timeManager);
	if (LN_Pred()->LN_Pred())
		prev = LN_Pred()->m_time;

	while (ignition->LN_Succ()) {
		if ((ignition->m_ignitionTime > prev) && (ignition->m_ignitionTime <= m_time)) {
			addIgnition(ignition);
			added = true;
		}
		ignition = ignition->LN_Succ();
	}
	return added;
}


template<class _type>
bool ScenarioTimeStep<_type>::findFirstIgnitionCentroid(XY_PointTempl<_type>& centroid) const {
	if (!m_scenario->m_scenario->m_impl->m_ignitionList.GetCount())
		return false;
	IgnitionNode<_type>* ignition = m_scenario->m_scenario->m_impl->m_ignitionList.LH_Head();
	WTime itime(ignition->m_ignitionTime);
	ignition = ignition->LN_Succ();

	double cnt = 0;
	while (ignition->LN_Succ()) {
		if (ignition->m_ignitionTime < itime)
			itime = ignition->m_ignitionTime;
		ignition = ignition->LN_Succ();
	}

	ignition = m_scenario->m_scenario->m_impl->m_ignitionList.LH_Head();
	while (ignition->LN_Succ()) {
		if (ignition->m_ignitionTime == itime) {
			XY_Point c;
			std::uint32_t i_cnt;
			ignition->m_ignitionCOM->GetIgnitionCount(&i_cnt);
			for (std::uint32_t i = 0; i < i_cnt; i++) {
				ignition->m_ignitionCOM->GetIgnitionCentroid(i, &c);
				centroid += c;
				cnt++;
			}
		}
		ignition = ignition->LN_Succ();
	}

	if (cnt > 1.0) {
		centroid.x /= cnt;
		centroid.y /= cnt;
	}
	return true;
}


template<class _type>
void ScenarioFire<_type>::AddFireFront(FireFront<_type> *ff) {

    #ifdef DEBUG
	weak_assert(!ff->m_fire);
    #endif

	ff->m_fire = this;
	AddPoly(ff);
}


template<class _type>
void ScenarioTimeStep<_type>::addIgnition(IgnitionNode<_type> *ignition) {			// this will need more work to clip it against other fires and breaks
																					// correctly
	std::uint32_t i_cnt, cnt = 0, max_size;
	if (FAILED(ignition->m_ignitionCOM->GetIgnitionCount(&cnt)))
		return;
	if (FAILED(ignition->m_ignitionCOM->GetIgnitionSize((std::uint32_t)-1, &max_size)))
		return;
	_type ignitionSize = m_scenario->m_scenario->m_ignitionSize;
	m_scenario->toInternal1D(ignitionSize);
	_type epsilon = EPSILON;		// epsilon is 5cm
	m_scenario->gridToInternal1D(epsilon);

	// need to ensure that ignitionSize is large enough; if plot resolution becomes too great then small ignitions
	// begin to vanish, as their individual points begin to appoximate one another..
	_type minIgnitionSize = epsilon * (m_scenario->m_scenario->m_initialVertexCount) * 0.16333 * 2.0;
	if (ignitionSize < minIgnitionSize)
	{
		// this ignition is currently too small - lets make it just barely big enough!
		weak_assert(8 <= m_scenario->m_scenario->m_initialVertexCount); // this approximation is only valid for n >= 8
		ignitionSize = minIgnitionSize;
	}

	XY_Poly ig(max_size);
	
	for (i_cnt = 0; i_cnt < cnt; i_cnt++) {
		XYPolySetType start;
		XY_PolyLL_Set<FireFront<_type>, _type> new_set;
		if (!(m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
			new_set.SetCacheScale(m_scenario->resolution());
		ScenarioFire<_type> *sf = nullptr;
		FireFront<_type> *fs;

		std::uint16_t ignitionType;
		std::uint32_t ii_cnt;
		ignition->m_ignitionCOM->GetIgnition(i_cnt, &ignitionType, &ig);

		if (m_scenario->m_scenario->m_optionFlags & ((1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING) | (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN))) {
			for (std::uint32_t ii = 0; ii < ig.NumPoints(); ii++) {
				XY_Point xy = ig.GetPoint(ii);
				m_scenario->toInternal(xy);
				ig.SetPoint(ii, xy);
			}
		}
		if ((m_scenario->m_scenario->m_dx != 0.0) || (m_scenario->m_scenario->m_dy != 0.0)) {
			double scale = 1.0;
			m_scenario->toInternal1D(scale);
			XYPointType dd(m_scenario->m_scenario->m_dx, m_scenario->m_scenario->m_dy);
			dd.ScaleXY(scale, scale);
			ig.TranslateXY(dd);
		}

		if (ignitionType == CWFGM_FIRE_IGNITION_POINT) {
			ig.CleanPoly(0.0, XY_Poly::PolygonType::MULTIPOINT);
			for (std::uint32_t i = 0; i < ig.NumPoints(); i++) {
				bool valid;
				addPoint(ig.GetPoint(i), valid, start);
			}

			if (!start.IsEmpty()) {
				new_set.BufferPolygon(&start, ignitionSize, CONSTANTS_NAMESPACE::TwoPi<_type>() / (_type)m_scenario->m_scenario->m_initialVertexCount, nullptr, m_scenario->m_multithread);

				// need to subtract from this all vector breaks and existing fires
				sf = new ScenarioFire<_type>(this, ignition, nullptr);
				while (fs = new_set.RemHead()) {
					FirePoint<_type>* fp = fs->LH_Head();
					while (fp->LN_Succ()) {
						fp->m_prevPoint = nullptr;
						fp = fp->LN_Succ();
					}

					sf->AddFireFront(fs);
					sf->m_fireArea += fs->Area();
				}
			}
		} else if (ignitionType == CWFGM_FIRE_IGNITION_LINE) {
			XYPolyType igQ;
			igQ.SetNumPoints(ig.NumPoints());
			for (ULONG iii = 0; iii < ig.NumPoints(); iii++)
				igQ.SetPoint(iii, ig.GetPoint(iii));
			igQ.CleanPoly(0.0, XY_Poly::PolygonType::POLYLINE);
			start.AddPoly(igQ, false);
			new_set.BufferPolygon(&start, ignitionSize, CONSTANTS_NAMESPACE::TwoPi<_type>() / (_type)m_scenario->m_scenario->m_initialVertexCount, NULL, false);

											// need to subtract from this all vector breaks and existing fires
			sf = new ScenarioFire<_type>(this, ignition, nullptr);
			while (fs = new_set.RemHead()) {
				FirePoint<_type> *fp = fs->LH_Head();
				while (fp->LN_Succ()) {
					fp->m_prevPoint = nullptr;
					fp = fp->LN_Succ();
				}

				sf->AddFireFront(fs);
				sf->m_fireArea += fs->Area();
			}

		} else if (ignitionType == CWFGM_FIRE_IGNITION_POLYGON_OUT) {
			XYPolyType igQ;
			igQ.SetNumPoints(ig.NumPoints());
			for (ULONG iii = 0; iii < ig.NumPoints(); iii++)
				igQ.SetPoint(iii, ig.GetPoint(iii));
			fs = new FireFront<_type>(NULL, igQ);
			fs->m_publicFlags |= FireFront<_type>::Flags::INTERPRET_POLYGON;
			fs->CleanPoly(0.0, FireFront<_type>::Flags::INTERPRET_POLYGON);
			new_set.AddPoly(fs);
			if (!(m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
				fs->SetCacheScale(new_set.GetCacheScale());
			new_set.Unwind(true, false, nullptr, nullptr);

			sf = new ScenarioFire<_type>(this, ignition, nullptr);
			while (fs = new_set.RemHead()) {
				FirePoint<_type> *fp = fs->LH_Head();
				while (fp->LN_Succ()) {
					fp->m_prevPoint = nullptr;
					fp = fp->LN_Succ();
				}

				sf->AddFireFront(fs);
			}
			sf->m_fireArea = sf->Area();
											// need to subtract from these all vector breaks and existing fires

		} else if (ignitionType == CWFGM_FIRE_IGNITION_POLYGON_IN) {
			XYPolyType igQ;
			igQ.SetNumPoints(ig.NumPoints());
			for (ULONG iii = 0; iii < ig.NumPoints(); iii++)
				igQ.SetPoint(iii, ig.GetPoint(iii));
			fs = new FireFront<_type>(NULL, igQ);
			fs->m_publicFlags |= FireFront<_type>::Flags::INTERPRET_POLYGON | FireFront<_type>::Flags::INTERIOR_SPECIFIED;
			fs->CleanPoly(0.0, FireFront<_type>::Flags::INTERPRET_POLYGON);
			new_set.AddPoly(fs);
			if (!(m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)))
				fs->SetCacheScale(new_set.GetCacheScale());
			new_set.Unwind(true, false, nullptr, nullptr);

			if (!m_fires.GetCount())
				sf = new ScenarioFire<_type>(this, ignition, nullptr);
			else {
				sf = m_fires.LH_Tail();
				auto poly = new_set.LH_Head();
				auto point = poly->LH_Head();	// this is an interior polygon so it should be associated with an exterior polygon
												// and if so, then it should be the last exterior polygon we encountered.  But if not,
												// then go looking for it, and if we can't find an exterior polygon to include it, then
												// let it calculate on its own
				while (!sf->PointInArea(*point, 0.0)) {
					weak_assert(false);
					sf = sf->LN_Pred();
				}
				if (!sf->LN_Pred())
					sf = new ScenarioFire<_type>(this, ignition, NULL);
			}
			if (fs = new_set.RemHead()) {
				fs->ReverseRotation();
				FirePoint<_type> *fp = fs->LH_Head();
				while (fp->LN_Succ()) {
					fp->m_prevPoint = nullptr;
					fp = fp->LN_Succ();
				}

				sf->AddFireFront(fs);
			}
			sf->m_fireArea = sf->Area();
		}

		if (sf) {
			ActiveFire<_type>* af;
			if (!m_fires.NodeHasIndex(sf)) {
				m_fires.AddTail(sf);
				af = new ActiveFire<_type>();
				m_scenario->m_activeFires.AddTail(af);
				af->LN_Ptr(sf);
				sf->m_activeFire = af;
				af->m_advanced = 1;
			} else
				af = sf->m_activeFire;

			if (m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS)) {
				_type area = 0.0;
				ScenarioFire<_type> *sf1 = m_fires.LH_Head();
				while (sf1->LN_Succ()) {
					_type a(sf1->Area());
					if (a > area)
						area = a;
					sf1 = sf1->LN_Succ();
				}
				m_scenario->fromInternal2D(area);
				area = area / 10000.0;//Ha
				_type st = m_scenario->m_scenario->spatialThreshold(area);
				m_scenario->gridToInternal1D(st);

				sf1 = m_fires.LH_Head();
				bool done = false;
				while ((!done) && (sf1->LN_Succ())) {
					if (sf != sf1) {
						if ((sf1->FastCollisionTest(*sf, st))) {
							weak_assert(sf1->m_activeFire->LN_Ptr() != sf1);
							FireFront<_type>	*ff = sf1->LH_Head(),
												*ff1 = sf->LH_Head();
							while ((!done) && (ff->LN_Succ())) {
								while ((!done) && (ff1->LN_Succ())) {
									if (ff->FastCollisionTest(*ff1, st)) {
										FirePoint<_type> *fp = ff->LH_Head();
										while ((!done) && (fp->LN_Succ())) {
											if (ff1->WithinDistance(*fp, st * 2.0, false)) {
												weak_assert(sf1->m_activeFire->m_advanced);
												sf1->m_activeFire->LN_Ptr(sf1);
												sf1->m_activeFire->m_advanced = 1;
												weak_assert(!af->Attached(sf->m_activeFire));
												af->Attach(sf->m_activeFire);
												done = true;
											}
											fp = fp->LN_Succ();
										}
									}
									ff1 = ff1->LN_Succ();
								}
								ff = ff->LN_Succ();
							}
						}
					}
					sf1 = sf1->LN_Succ();
				}
			}
		}
	}
}


template<class _type>
void ScenarioTimeStep<_type>::addPoint(const XYPointType &loc, bool &valid, XYPolySetType &start)
{
	std::uint32_t ii, ii_cnt;
	if ((!m_scenario->IsNonFuel(m_time, loc, valid)) && (valid))
	{
		bool can_continue = true;
		if (m_scenario->StaticVectorBreakCount())
		{
			ii_cnt = m_scenario->StaticVectorBreakCount();
			for (ii = 0; ii < ii_cnt; ii++)
			{
				auto v = m_scenario->StaticVectorBreak(ii);
				if (v->box.PointInside(loc))
				{
					auto p = v->LH_Head();
					while (p->LN_Succ()) {
						if ((p->m_usedTime.GetTime(0)) || (p->FastCollisionTest(loc, 0.0))) {
							if (!p->m_usedTime.GetTime(0)) {				// if we happen to come across a vector break that could include the new ignition point, then flag it as
								p->m_usedTime = m_time;						// something we find interesting

								XY_PolyLLPolyRef<_type>* r = new XY_PolyLLPolyRef<_type>();
								r->LN_Ptr(p);
								m_staticVectorBreaksLL.AddTail(r);
							}

							if (p->PointInArea(loc, 0.0)) {
								can_continue = false;
								break;
							}
						} else {
							weak_assert(!p->FastCollisionTest(loc, 0.0));
						}
						p = p->LN_Succ();
					}
				}
				if (!can_continue)
					break;
			}
		}
		if ((can_continue) && (m_vectorBreaksLL))
		{
			if (m_vectorBreaksLL->size())
			{
				ii_cnt = (std::uint32_t)m_vectorBreaksLL->size();
				for (ii = 0; ii < ii_cnt; ii++)
				{
					if ((*m_vectorBreaksLL)[ii]->PointInArea(loc, 0.0))
					{
						can_continue = false;
						break;
					}
				}
			}
		}
		if ((can_continue) && (m_scenario->AssetCount())) {
			AssetNode<_type>* an = m_scenario->m_scenario->m_impl->m_assetList.LH_Head();
			//cutout if the asset operation count is 1 (a single point) or -1 (wait for all points)
			bool globalCutout = (m_scenario->m_scenario->m_globalAssetOperation == 1) || (m_scenario->m_scenario->m_globalAssetOperation == (std::uint32_t)-1);
			while (an->LN_Succ()) {
				AssetGeometryNode<_type>* agn = an->m_geometry.LH_Head();
				while (agn->LN_Succ()) {
					if (!agn->m_arrived && (agn->m_geometry.IsPolygon())) {
						if (agn->m_geometry.PointInArea(loc, 0.0)) {
							agn->m_arrivalTime = m_scenario->m_scenario->m_startTime;
							agn->m_arrived = true;
							if (an->m_operation == 1 || (an->m_operation == (std::uint32_t)-1) || globalCutout) {
							    can_continue = false;
								agn->m_closestFirePoint = GetNearestPoint(loc, true, &agn->m_closestFireFront, true);
								agn->m_closestPoint.copyValuesFrom(*agn->m_closestFirePoint);
								agn->fixClosestPoint();
								break;
						    }
							else {
								m_assetCount++;
							}
						}
					}
					agn = agn->LN_Succ();
				}
				if (!can_continue)
					break;
				an = an->LN_Succ();
			}
		}
		if (can_continue)
		{
			ActiveFire<_type> *af = m_scenario->m_activeFires.LH_Head();
			std::int32_t val = 0;
			std::int32_t on_hull = 0;
			std::int32_t retval;
			while (af->LN_Succ())
			{
				if (af->LN_Ptr()) {	// this can happen if the fire's area drops to 0 or is otherwise removed because of the untangler
					retval = af->LN_Ptr()->PointInArea(loc, 0.0);
					val += (retval & (~1));
					on_hull |= (retval & 1);
				}
				af = af->LN_Succ();
			}
			std::int32_t inside = (val + on_hull) & (~1);
			if (!inside)
			{
				XYPolyConstType pc(&loc, 1);
				start.AddPoly(pc, false);
			}
		}
	}
}


template<class _type>
struct stepRescan {
	FireFront<_type>* VOLATILE ff;

	CThreadSemaphore m_lock;
	_type epsilon;
};


template<class _type>
std::uint32_t AFX_CDECL stepAddPoints(APTR parameter) {
	stepRescan<_type> *sr = (struct stepRescan<_type>*)parameter;

	while (1) {
		sr->m_lock.Lock();

		if (!sr->ff) {
			sr->m_lock.Unlock();
			return 1;
		}
		FireFront<_type> *ff = (FireFront<_type>*)sr->ff;
		if (!ff->LN_Succ()) {

			const ScenarioFire<_type> *sf = ff->LN_Pred()->Fire();
			do {
				sf = sf->LN_Succ();
				if (!sf->LN_Succ()) {
					sr->ff = NULL;
					sr->m_lock.Unlock();
					return 1;
				}
			} while (!sf->NumPolys());
			ff = sf->LH_Head();				// ff is what we're going to work on
		} 

		sr->ff = ff->LN_Succ();					// set the pointer to the next one

		sr->m_lock.Unlock();

		ff->CleanPoly(sr->epsilon, FireFront<_type>::Flags::INTERPRET_POLYGON);
		ff->AddPoints();			
	}
}


template<class _type>
void ScenarioTimeStep<_type>::AddFirePoints() {
	_type epsilon = EPSILON;		// epsilon is 5cm
	m_scenario->gridToInternal1D(epsilon);

	if ((m_scenario->m_multithread) && (m_fires.GetCount() > 1)) {
		ScenarioFire<_type> *sf = m_fires.LH_Head();
		while (sf->LN_Succ()) {
			std::vector<FireFront<_type>*> &fp_array = m_scenario->m_omp_ff_array;
			std::int32_t i = 0, num_pts = sf->NumPolys();
			if (fp_array.size() < num_pts)
				fp_array.resize(num_pts);
			FireFront<_type> *fp1 = sf->LH_Head();
			while (fp1->LN_Succ()) {
				fp_array[i++] = fp1;
				fp1 = fp1->LN_Succ();
			}
			if (m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC))
				sf->InitArea(sf->Area());

			int thread_id = (m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY))) ? -1 : -2;
			#pragma omp parallel for num_threads(m_scenario->m_scenario->m_threadingNumProcessors) firstprivate(thread_id)
			for (i = 0; i < num_pts; i++) {
				if (thread_id == -1) {
					thread_id = omp_get_thread_num();
					CWorkerThread::native_handle_type thread = CWorkerThreadPool::GetCurrentThread();
					CWorkerThreadPool::SetThreadAffinityToMask(thread, thread_id);
				}
				FireFront<_type> *fs = fp_array[i];
				fs->CleanPoly(epsilon, FireFront<_type>::Flags::INTERPRET_POLYGON);
				fs->AddPoints();			
			}

			sf = sf->LN_Succ();
		}

		sf = m_fires.LH_Head();
		while (sf->LN_Succ()) {
			FireFront<_type> *fs = sf->LH_Head();
			while (fs->LN_Succ()) {
				if (fs->NumPoints() < 3) {				// AddPoints may remove points - when the line folds back on itself for no good reason - 070309 - RWB found
					FireFront<_type> *next_fs = fs->LN_Succ();		// that on occasion, the fire may end up completely "empty" in which case it should be removed!
					sf->RemovePoly(fs);
					delete fs;
					fs = next_fs;
				} else	fs = fs->LN_Succ();
			}
			sf = sf->LN_Succ();
		}

	} else {
		ScenarioFire<_type> *sf = m_fires.LH_Head();
		while (sf->LN_Succ()) {
			if (m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC))
				sf->InitArea(sf->Area());
			FireFront<_type> *fs = sf->LH_Head();
			while (fs->LN_Succ()) {
				fs->CleanPoly(epsilon, FireFront<_type>::Flags::INTERPRET_POLYGON);

				fs->AddPoints();
				if (fs->NumPoints() < 3) {				// AddPoints may remove points - when the line folds back on itself for no good reason - 070309 - RWB found
					FireFront<_type> *next_fs = fs->LN_Succ();		// that on occasion, the fire may end up completely "empty" in which case it should be removed!
					sf->RemovePoly(fs);
					delete fs;
					fs = next_fs;
				} else	fs = fs->LN_Succ();
			}

			sf = sf->LN_Succ();
		}
	}
}


template<class _type>
std::uint32_t ScenarioFire<_type>::NumActivePoints() const {
	std::int32_t cnt = 0;
	FireFront<_type> *ff = LH_Head();
	while (ff->LN_Succ()) {
		cnt += ff->NumActivePoints();
		ff = ff->LN_Succ();
	}
	return cnt;
}


template<class _type>
std::uint32_t ScenarioTimeStep<_type>::NumActivePoints() const {
	std::int32_t cnt = 0;
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		cnt += sf->NumActivePoints();
		sf = sf->LN_Succ();
	}
	return cnt;
}


template<class _type>
HRESULT ScenarioFire<_type>::RetrieveStat(const std::uint16_t stat, double *stats) const {
	double s = 0.0;
	HRESULT hr;
	if (stat == CWFGM_FIRE_STAT_NUM_FRONTS)	{
		*stats = NumPolys();
		return S_OK;
	}
	*stats = 0.0;
	FireFront<_type> *ff = LH_Head();
	FirePoint<_type> *fp;
	while (ff->LN_Succ()) {
		if (stat != CWFGM_FIRE_STAT_NUM_ACTIVE_FRONTS)
			if (FAILED(hr = ff->RetrieveStat(stat, &s)))
				return hr;

		switch (stat) {
			case CWFGM_FIRE_STAT_AREA:		*stats = m_fireArea;
								return S_OK;

			case CWFGM_FIRE_STAT_ACTIVE_PERIMETER:
			case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER:
			case CWFGM_FIRE_STAT_TOTAL_PERIMETER:
			case CWFGM_FIRE_STAT_NUM_POINTS:
			case CWFGM_FIRE_STAT_NUM_ACTIVE_POINTS:	*stats += s;
								break;

			case CWFGM_FIRE_STAT_ROS:
			case CWFGM_FIRE_STAT_CFB:
			case CWFGM_FIRE_STAT_CFC:
			case CWFGM_FIRE_STAT_SFC:
			case CWFGM_FIRE_STAT_TFC:
			case CWFGM_FIRE_STAT_FI:
			case CWFGM_FIRE_STAT_HFI:
			case CWFGM_FIRE_STAT_HCFB:
			case CWFGM_FIRE_STAT_FLAMELENGTH:
			case CWFGM_FIRE_STAT_MAXIMUM_BURN_DISTANCE:	
								if (*stats < s)
									*stats = s;
								break;

			case CWFGM_FIRE_STAT_NUM_ACTIVE_FRONTS:	fp = ff->LH_Head();
								while (fp->LN_Succ()) {
									if (!fp->m_status) {
										(*stats)++;
										break;
									}
									fp = fp->LN_Succ();
								}
								break;

			default:			return ERROR_FIRE_STAT_UNKNOWN;
		}
		ff = ff->LN_Succ();
	}
	return S_OK;
}


template<class _type>
HRESULT ScenarioTimeStep<_type>::RetrieveStat(const std::uint16_t stat, double *stats) const {
	double s = 0.0;
	HRESULT hr;
	*stats = 0.0;
	if ((stat == CWFGM_FIRE_STAT_CUMULATIVE_NUM_POINTS) || (stat == CWFGM_FIRE_STAT_CUMULATIVE_NUM_ACTIVE_POINTS)) {
		std::uint16_t _stat;
		if (stat == CWFGM_FIRE_STAT_CUMULATIVE_NUM_POINTS)
			_stat = CWFGM_FIRE_STAT_NUM_POINTS;
		else
			_stat = CWFGM_FIRE_STAT_NUM_ACTIVE_POINTS;

		const ScenarioTimeStep<_type> *sts = this;
		while (sts->LN_Pred()) {
			double stat_c;
			sts->RetrieveStat(_stat, &stat_c);
			*stats += stat_c;
			sts = sts->LN_Pred();
		}
		}
	else {
		ScenarioFire<_type>* sf = m_fires.LH_Head();
		while (sf->LN_Succ()) {
			if (FAILED(hr = sf->RetrieveStat(stat, &s)))
				return hr;

			switch (stat) {
			case CWFGM_FIRE_STAT_AREA:
			case CWFGM_FIRE_STAT_ACTIVE_PERIMETER:
			case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER:
			case CWFGM_FIRE_STAT_TOTAL_PERIMETER:
			case CWFGM_FIRE_STAT_NUM_FRONTS:
			case CWFGM_FIRE_STAT_NUM_ACTIVE_FRONTS:
			case CWFGM_FIRE_STAT_NUM_POINTS:
				case CWFGM_FIRE_STAT_NUM_ACTIVE_POINTS:	*stats += s;
				break;

			case CWFGM_FIRE_STAT_ROS:
			case CWFGM_FIRE_STAT_CFB:
			case CWFGM_FIRE_STAT_CFC:
			case CWFGM_FIRE_STAT_SFC:
			case CWFGM_FIRE_STAT_TFC:
			case CWFGM_FIRE_STAT_FI:
			case CWFGM_FIRE_STAT_HFI:
			case CWFGM_FIRE_STAT_HCFB:
			case CWFGM_FIRE_STAT_FLAMELENGTH:
				case CWFGM_FIRE_STAT_MAXIMUM_BURN_DISTANCE:	if (*stats < s)
																*stats = s;
															break;

			default:			return ERROR_FIRE_STAT_UNKNOWN;
			}
			sf = sf->LN_Succ();
		}
	}
	return S_OK;
}


template<class _type>
HRESULT ScenarioTimeStep<_type>::RetrieveStat(const std::uint16_t stat, PolymorphicAttribute *stats) const {
	double s = 0.0, ss = 0.0;
	HRESULT hr;
	*stats = 0.0;
	XY_PointTempl<_type> pt;
	std::uint32_t flags;
	WTime time(m_time);
	NumericVariant timespan;
	bool time_valid;
	grid::AttributeValue timespan_valid;

	if ((stat == CWFGM_FIRE_STAT_SUNRISE) ||
		(stat == CWFGM_FIRE_STAT_SUNSET) ||
		(stat == CWFGM_FIRE_STAT_SOLARNOON) ||
		(stat == CWFGM_FIRE_STAT_SUNRISE_CENTROID_X) ||
		(stat == CWFGM_FIRE_STAT_SUNRISE_CENTROID_Y) ||
		(stat == CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_PERIOD_START_COMPUTED) ||
		(stat == CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_PERIOD_END_COMPUTED)) {
		switch (stat) {
			case CWFGM_FIRE_STAT_SUNRISE:
			case CWFGM_FIRE_STAT_SUNSET:
			case CWFGM_FIRE_STAT_SOLARNOON:
			case CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_PERIOD_START_COMPUTED:
			case CWFGM_GRID_ATTRIBUTE_BURNINGCONDITION_PERIOD_END_COMPUTED:
				if (!(const_cast<ScenarioTimeStep<_type> *>(this))->Centroid(&pt))
					//		go and get the prior timestep's centroid
					if (!findFirstIgnitionCentroid(pt))
						// get the centroid of the grid dataset
						pt = m_scenario->start_center();
				if (stat == CWFGM_FIRE_STAT_SUNRISE)
					flags = CWFGM_GETEVENTTIME_FLAG_SEARCH_SUNRISE;
				else if (stat == CWFGM_FIRE_STAT_SUNSET)
					flags = CWFGM_GETEVENTTIME_FLAG_SEARCH_SUNSET;
				else if (stat == CWFGM_FIRE_STAT_SOLARNOON)
					flags = CWFGM_GETEVENTTIME_FLAG_SEARCH_SOLARNOON;
				else {
					if (SUCCEEDED(hr = m_scenario->m_scenario->m_gridEngine->GetAttributeData(m_scenario->m_scenario->m_layerThread, pt, m_time, WTimeSpan(0), stat, 0, &timespan, &timespan_valid, nullptr)) &&
						(timespan_valid != grid::AttributeValue::NOT_SET)) {
						std::int64_t s_time;
						if (!(variantToInt64(timespan, &s_time))) {
							weak_assert(false);
							return E_INVALIDARG;
						}
						*stats = s_time;
						return S_OK;
					}
					PolymorphicAttribute pa;
					*stats = pa;
					return S_OK;
				}
				m_scenario->m_scenario->m_gridEngine->GetEventTime(m_scenario->m_scenario->m_layerThread, pt, flags, m_time, &time, &time_valid);
				if (time_valid)
					*stats = time.GetTotalSeconds();
				else {
					PolymorphicAttribute pa;
					*stats = pa;
				}
				return S_OK;

			case CWFGM_FIRE_STAT_SUNRISE_CENTROID_X:
			case CWFGM_FIRE_STAT_SUNRISE_CENTROID_Y:
				if (!(const_cast<ScenarioTimeStep<_type> *>(this))->Centroid(&pt))
					//		go and get the prior timestep's centroid
					if (!findFirstIgnitionCentroid(pt))
						// get the centroid of the grid dataset
						pt = m_scenario->start_center();
				if (stat == CWFGM_FIRE_STAT_SUNRISE_CENTROID_X)
					*stats = pt.x;
				else
					*stats = pt.y;
				return S_OK;
		}
	}
	double _s;
	hr = RetrieveStat(stat, &_s);
	if (SUCCEEDED(hr))
		*stats = _s;
	return hr;
}


template<class _type>
HRESULT ScenarioFire<_type>::RetrieveStat(std::uint16_t stat, double greater_equal, double less_than, double *stats) const {
	HRESULT hr;
	*stats = 0.0;
	FireFront<_type> *ff = LH_Head();
	while (ff->LN_Succ()) {
		double s;
		if (FAILED(hr = ff->RetrieveStat(stat, greater_equal, less_than, &s)))
			return hr;
		*stats = *stats + s;
		ff = ff->LN_Succ();
	}
	return S_OK;
}


template<class _type>
void ScenarioTimeStep<_type>::retrieveDStat(std::uint16_t stat, ICWFGM_Fuel* fuel, std::uint16_t discretize, ScenarioTimeStep<_type>* sts_prev, double* stats, double step,
	const std::uint64_t cx, const std::uint64_t cy, boost::multi_array<typename Scenario<_type>::closest_calc, 2>& pt_array, CThreadSemaphore* add_lock) const {
	{
		XYPointType pt((cx + 0.5) * m_scenario->resolution() + m_curr_ll.x, (cy + 0.5) * m_scenario->resolution() + m_curr_ll.y);
		m_scenario->toInternal(pt);
		bool valid;
		ICWFGM_Fuel* fuel2 = m_scenario->GetFuel(m_time, pt, valid);
		if ((!valid) || (!fuel2))
			return;

		CCWFGM_FuelOverrides overrides;
		m_scenario->GetCorrectedFuel(pt, m_time, fuel2, overrides);

		bool result = false;
		HRESULT hr;
		if ((FAILED(hr = fuel2->IsNonFuel(&result))) || (result))
			return;

		if (fuel) {
			if (fuel2 != fuel)
				return;
		}
	}
	std::uint32_t x, y;
	for (x = 0; x < discretize; x++)
		for (y = 0; y < discretize; y++) {
			pt_array[x][y].ff = nullptr;
			pt_array[x][y].fp = nullptr;
			pt_array[x][y].time = 0;
		}
	bool found = false;

	{
		XYPointType p;
		p.x = (_type)(cx * m_scenario->resolution()) + step * 0.5 + m_curr_ll.x;
		p.y = (_type)(cy * m_scenario->resolution()) + step * 0.5 + m_curr_ll.y;
		m_scenario->toInternal(p);
		m_scenario->toInternal1D(step);
		_type py = p.y;
		for (x = 0; x < discretize; x++, p.x += step) {
			p.y = py;
			for (y = 0; y < discretize; y++, p.y += step) {
				if ((stat != CWFGM_FIRE_STAT_BURNED) && (sts_prev)) {
					if (sts_prev->PointInArea(p))
						continue;
				}
				if (!PointInArea(p))
					continue;
				FireFront<_type>* closest_ff;
				FirePoint<_type>* closest = GetNearestPoint(p, true, &closest_ff, true);

				LONG inside_break = 0;
				if (sts_prev)
					inside_break = sts_prev->pointInsideVectors(p);
				else
					inside_break = pointInsideVectors(p);

				if (!inside_break)
					{
					FirePoint<_type>* prev_closest;
					FireFront<_type>* prev_closest_ff;
					if (sts_prev)	prev_closest = sts_prev->GetNearestPoint(p, false, &prev_closest_ff, false);
					else			prev_closest = nullptr;

					if ((prev_closest) ) {
						// the other part of the above if statement is commented out so we only use vertices from the prior step, going forward,
						// rather than before or forward in time - this is different from the other 

						pt_array[x][y].fp = prev_closest;
						pt_array[x][y].ff = prev_closest_ff;
						pt_array[x][y].time = sts_prev->m_time.GetTime(0);
					}
					else {
						pt_array[x][y].fp = closest;
						pt_array[x][y].ff = closest_ff;
						pt_array[x][y].time = m_time.GetTime(0);
					}
					found = true;
				}
			}
		}
	}

	if (found) {
		CThreadSemaphoreEngage a_d(add_lock, SEM_TRUE);
		for (x = 0; x < discretize; x++)
			for (y = 0; y < discretize; y++) {
				if (pt_array[x][y].fp) {
					if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED))
						(*stats) += 1.0;
					else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
						double tfc;
						HRESULT hr = pt_array[x][y].fp->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(tfc));
#else
						weak_assert(!_isnan(tfc));
#endif
						(*stats) += tfc;
					} else if (stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
						double sfc;
						HRESULT hr = pt_array[x][y].fp->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(sfc));
#else
						weak_assert(!_isnan(sfc));
#endif
						(*stats) += sfc;
					} else if (stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
						double cfc;
						HRESULT hr = pt_array[x][y].fp->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(cfc));
#else
						weak_assert(!_isnan(cfc));
#endif
						(*stats) += cfc;
					}
				}
			}
	}
}


template<class _type>
HRESULT ScenarioTimeStep<_type>::RetrieveStat(const std::uint16_t stat, ICWFGM_Fuel* fuel, const std::uint16_t discretize, ScenarioTimeStep<_type>* sts_prev, double* stats) const {
	*stats = 0.0;
	XYRectangleType bbox;
	if (!BoundingBox(bbox))
		return ERROR_SCENARIO_NO_FIRES;

	CThreadSemaphore add_lock;
	const _type step = m_scenario->resolution() / (_type)discretize;

	m_scenario->fromInternal(bbox.m_min);
	m_scenario->fromInternal(bbox.m_max);

	const std::int64_t cx1 = m_scenario->toGridScaleX(bbox.m_min);
	const std::int64_t cy1 = m_scenario->toGridScaleY(bbox.m_min);
	const std::int64_t cx2 = m_scenario->toGridScaleX(bbox.m_max);
	const std::int64_t cy2 = m_scenario->toGridScaleY(bbox.m_max);

	const std::uint64_t xsize(cx2 - cx1 + 1), ysize(cy2 - cy1 + 1);
	const std::uint64_t num(xsize * ysize);
	int count;
	if (((num * discretize * discretize)) > (std::uint64_t)10)
		count = m_scenario->m_scenario->m_threadingNumProcessors;
	else
		count = 1;

	boost::multi_array<typename Scenario<_type>::closest_calc, 2> pt_array(boost::extents[discretize][discretize]);
	if (count != 1) {
		int thread_id = (m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY))) ? -1 : -2;
		std::int64_t i;

#pragma omp parallel for num_threads(m_scenario->m_scenario->m_threadingNumProcessors) firstprivate(thread_id, pt_array)
		for (i = 0; i < num; i++) {
			if (thread_id == -1) {
				thread_id = omp_get_thread_num();
				CWorkerThread::native_handle_type thread = CWorkerThreadPool::GetCurrentThread();
				CWorkerThreadPool::SetThreadAffinityToMask(thread, thread_id);
			}
			const uint64_t cx = cx1 + i % xsize;
			const uint64_t cy = cy1 + i / xsize;
			retrieveDStat(stat, fuel, discretize, sts_prev, stats, step, cx, cy, pt_array, &add_lock);
		}
	} else {
		std::uint64_t cx, cy;
		for (cx = cx1; cx <= cx2; cx++)
			for (cy = cy1; cy <= cy2; cy++) {
				retrieveDStat(stat, fuel, discretize, sts_prev, stats, step, cx, cy, pt_array, nullptr);
			}
	}

	if ((*stats) != 0.0) {
		if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED)) {
			(*stats) *= step;
			(*stats) *= step;
		}
		else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
			(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
			(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
			(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
			(*stats) *= step;
			(*stats) *= step;	// convert it to kg, and for # meters that this sub-cell represents

			if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
				const double HeatOfCombustion = 18000.0; // kJ/kg
				(*stats) *= HeatOfCombustion;
				WTime mtime(m_scenario->m_scenario->m_startTime);
				if ((sts_prev) && (mtime < sts_prev->m_time))
					mtime = sts_prev->m_time;
				WTimeSpan delta = m_time - mtime;
				(*stats) /= delta.GetSecondsFraction();
			}
		}
	}
	return S_OK;
}


template<class _type>
double ScenarioFire<_type>::MinimumROSRatio() const {
	double min_ratio = 1.0;
	FireFront<_type> *ff = LH_Head();
	while (ff->LN_Succ()) {
		weak_assert(ff->Fire());
		double ratio = ff->MinimumROSRatio();
		if (ratio < min_ratio)
			min_ratio = ratio;
		ff = ff->LN_Succ();
	}
	return min_ratio;
}


template<class _type>
double ScenarioFire<_type>::MaximumROS() const {
	double max_ros = 0.0;
	FireFront<_type> *ff = LH_Head();
	while (ff->LN_Succ()) {
		double m_ros = ff->MaximumROS();
		if (m_ros > max_ros)
			max_ros = m_ros;
		ff = ff->LN_Succ();
	}
	return max_ros;
}


template<class _type>
double ScenarioFire<_type>::MaximumCardinalROS() const {
	double max_ros = 0.0;
	FireFront<_type> *ff = LH_Head();
	while (ff->LN_Succ()) {
		double m_ros = ff->MaximumCardinalROS();
		if (m_ros > max_ros)
			max_ros = m_ros;
		ff = ff->LN_Succ();
	}
	return max_ros;
}


template<class _type>
bool ScenarioTimeStep<_type>::canBurn() const {
	ScenarioFire<_type>* sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		if (sf->m_canBurn)
			return true;
		sf = sf->LN_Succ();
	}
	return false;
}


template<class _type>
void ScenarioFire<_type>::CalculateEndTime() {
	weak_assert(m_activeFire->LN_Ptr() == this);
	const WTime step_start(TimeStep()->m_time);

	if (step_start == m_activeFire->m_startTime)		// was already calculated but not used
		return;

	double ratio = MinimumROSRatio();
	bool during_acceleration;
	if (ratio < 0.90)		during_acceleration = true;
	else					during_acceleration = false;

	WTimeSpan temporalThreshold;
	if (during_acceleration) {
		temporalThreshold = TimeStep()->m_scenario->m_scenario->m_temporalThreshold_Accel;
		if (temporalThreshold.GetTotalSeconds() == 0)
			temporalThreshold = WTimeSpan(0, 0, 2, 0);
		else if (temporalThreshold.GetTotalSeconds() == (std::int64_t)-1)
			temporalThreshold = WTimeSpan(0, 1, 0, 0);
	}
	else
		temporalThreshold = WTimeSpan(0, 1, 0, 0);

	double min_duration;
	double max_ros;

	XY_PointTempl<_type> centroid;
	(const_cast<ScenarioTimeStep<_type> *>(TimeStep()))->Centroid(&centroid);
	if (TimeStep()->m_scenario->CanBurn(step_start, centroid)) {
		double by_distance_duration, by_time_duration;
		if (TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_CARDINAL_ROS))
			max_ros = MaximumCardinalROS();
		else
			max_ros = MaximumROS();				// maximum meters per minute

		bool dynamic = (TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC)) ? true : false;

		if (max_ros > 0.0) {
			if (dynamic) {
				double min_time = -1.0;
				double area = Area();//m^2
				TimeStep()->m_scenario->fromInternal2D(area);
				area = area / 10000.0;//Ha
				double st = TimeStep()->m_scenario->m_scenario->spatialThreshold(area);
				TimeStep()->m_scenario->gridToInternal1D(st);
				TimeStep()->m_scenario->fromInternal1D(st);
				double time = st / max_ros * 60.0;
				by_distance_duration = time;
			}
			else {
				double max_grid_dist = TimeStep()->m_scenario->m_scenario->spatialThreshold(0.0);
				TimeStep()->m_scenario->gridToInternal1D(max_grid_dist);
				TimeStep()->m_scenario->fromInternal1D(max_grid_dist);
				by_distance_duration = max_grid_dist / max_ros * 60.0;
			}
		}
		else
			by_distance_duration = (double)temporalThreshold.GetTotalSeconds();

		if (temporalThreshold.GetTotalSeconds() > 0)
			by_time_duration = (double)temporalThreshold.GetTotalSeconds();
		else
			by_time_duration = by_distance_duration;

		min_duration = min(by_distance_duration, by_time_duration);
	}
	else {
		min_duration = (double)temporalThreshold.GetTotalSeconds();
		m_canBurn = false;
	}

	WTimeSpan calculation_step = WTimeSpan((std::int64_t)min_duration);

	m_activeFire->m_startTime = step_start;
	m_activeFire->m_endTime = step_start + calculation_step;
}


template<class _type>
HRESULT ScenarioTimeStep<_type>::RetrieveStat(const std::uint16_t stat, const double greater_equal, const double less_than, double *stats) const {
	HRESULT hr;
	*stats = 0.0;
	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		double s;
		if (FAILED(hr = sf->RetrieveStat(stat, greater_equal, less_than, &s)))
			return hr;
		*stats = *stats + s;
		sf = sf->LN_Succ();
	}
	return S_OK;
}


template<class _type>
void ScenarioTimeStep<_type>::RecordActiveFires() {
	ActiveFire<_type>* caf, * af = m_scenario->m_activeFires.LH_Head(), * caf1, * af1;
	while (af->LN_Succ()) {
		caf = new ActiveFire<_type>(af);
		m_activeFiresState.AddTail(caf);
		af = af->LN_Succ();
	}

	af = m_scenario->m_activeFires.LH_Head();
	caf = m_activeFiresState.LH_Head();
	while (af->LN_Succ()) {
		af1 = af->LN_Succ();
		caf1 = caf->LN_Succ();
		while (af1->LN_Succ()) {
			if (af->Attached(af1) && (!caf->Attached(caf1)))
				caf->Attach(caf1);
			af1 = af1->LN_Succ();
			caf1 = caf1->LN_Succ();
		}
		af = af->LN_Succ();
		caf = caf->LN_Succ();
	}
}


template<class _type>
void ScenarioTimeStep<_type>::RestoreActiveFires() {
	ActiveFire<_type>* af = m_scenario->m_activeFires.LH_Head(), * caf, * af1, * caf1;
	while (af->LN_Succ()) {
		ActiveFire<_type>* caf = m_activeFiresState.LH_Head();
		bool found = false;
		while (caf->LN_Succ()) {
			if (caf->m_master == af) {
				found = true;
				break;
			}
			caf = caf->LN_Succ();
		}
		caf = af;									// re-use this variable for something we may want to delete, while incrementing our 'af' iterator
		af = af->LN_Succ();
		if (!found) {
			m_scenario->m_activeFires.Remove(caf);	// remove any active fires that were introduced in a subsequent step
			delete caf;
		}
	}

	if (m_activeFiresState.IsEmpty()) {
		weak_assert(m_scenario->m_activeFires.IsEmpty());
		return;
	}

	weak_assert(m_scenario->m_activeFires.GetCount() == m_activeFiresState.GetCount());

	af = m_scenario->m_activeFires.LH_Head();		// detach all associations
	while (af->LN_Succ()) {
		if (af->m_mate_next != af)
			af->Detach();
		af = af->LN_Succ();
	}

	af = m_scenario->m_activeFires.LH_Head();
	caf = m_activeFiresState.LH_Head();
	af->LN_Ptr(caf->LN_Ptr());
	af->m_startTime = caf->m_startTime;
	af->m_endTime = caf->m_endTime;
	af->m_boundingBox = caf->m_boundingBox;
	af->m_advanced = caf->m_advanced;

	weak_assert(af->LN_Ptr()->m_activeFire == af);

	while (af->LN_Succ()) {
		af1 = af->LN_Succ();
		caf1 = caf->LN_Succ();
		while (af1->LN_Succ()) {
			if ((!af->Attached(af1)) && (caf->Attached(caf1)))
				af->Attach(af1);					// rebuild associations among active fires, from the cache
			af1 = af1->LN_Succ();
			caf1 = caf1->LN_Succ();
		}
		af = af->LN_Succ();
		caf = caf->LN_Succ();
	}
}

#include "InstantiateClasses.cpp"
