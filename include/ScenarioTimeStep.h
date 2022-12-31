/**
 * WISE_Scenario_Growth_Module: ScenarioTimeStep.h
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

#ifndef __SCENARIOTIMESTEP_H
#define __SCENARIOTIMESTEP_H

#include "firefront.h"
#include "scenario.h"
#include "StopCondition.h"
#include <boost/multi_array.hpp>
#include <chrono>

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif

template<class _type>
class Scenario;

template<class _type>
class ScenarioFire : public MinNode, public XY_PolyLL_Set<FireFront<_type>, _type> {		// this class only exists (now) so we can keep a set of each fire front on a list, where each fire front
	using XYPolyLLType = XY_PolyLL_BaseTempl<_type>;
	using IntersectionType = XY_PolyIntersection<_type>;

	const ScenarioTimeStep<_type>	*m_timeStep;
	const IgnitionNode<_type>		*m_ignition;
	ScenarioFire<_type>				*m_calcPred,
									*m_calcSucc;
	_type							m_initArea;

public:
	using XY_PolyLL_Set<FireFront<_type>, _type>::LH_Head;
	using XY_PolyLL_Set<FireFront<_type>, _type>::Area;
	using XY_PolyLL_Set<FireFront<_type>, _type>::NumPolys;
	using XY_PolyLL_Set<FireFront<_type>, _type>::GetCacheScale;
	using XY_PolyLL_Set<FireFront<_type>, _type>::SetCacheScale;
	using XY_PolyLL_Set<FireFront<_type>, _type>::AddPoly;
	using XY_PolyLL_Set<FireFront<_type>, _type>::RemovePoly;

public:									// one polygon defining (part of) a specific fire - so all fire fronts for one ignition sit on this class
	DECLARE_OBJECT_CACHE_MT(ScenarioFire<_type>, ScenarioFire)

	ScenarioFire(const ScenarioTimeStep<_type> *timeStep, const IgnitionNode<_type> *ignition, ScenarioFire<_type> *pred);

	__INLINE ScenarioFire<_type> *LN_Succ() const				{ return (ScenarioFire<_type>*)MinNode::LN_Succ(); };
	__INLINE ScenarioFire<_type> *LN_Pred() const				{ return (ScenarioFire<_type>*)MinNode::LN_Pred(); };
	__INLINE ScenarioFire<_type> *LN_CalcPred() const			{ return m_calcPred; };	// can be NULL!
	__INLINE ScenarioFire<_type> *LN_CalcSucc() const			{ return m_calcSucc; };	// can be NULL!

	__INLINE const ScenarioTimeStep<_type> *TimeStep() const	{ return m_timeStep; };
	__INLINE const IgnitionNode<_type> *Ignition() const		{ return m_ignition; };

	_type						m_fireArea;					// used when running a simulation - it's a temporary used to rank fires by size
	class ActiveFire<_type>		*m_activeFire;
	std::uint32_t				m_newVertexStatus;
	std::uint32_t				m_canBurn : 1,
								m_bits;

	virtual FireFront<_type>*New() const override;
	virtual FireFront<_type>*NewCopy(const XYPolyLLType&toCopy) const override;

	bool AdvanceFire(const _type scale);
	void StatsFires();

	void AddFireFront(FireFront<_type> *ff);

	std::uint32_t NumActivePoints() const;
	HRESULT RetrieveStat(const std::uint16_t stat, double *stats) const;
	HRESULT RetrieveStat(const std::uint16_t stat, const double greater_equal, const double less_than, double*stats) const;
	double MaximumROS() const;
	double MaximumCardinalROS() const;
	double MinimumROSRatio() const;

	_type InitArea() const;
	void InitArea(_type area)									{ m_initArea = area; };

	void CalculateEndTime();

	void setCalcPred(ScenarioFire<_type> *sf)					{ m_calcPred = sf; }				// really shouldn't be called for anywhere other than very specific places, this
	void setCalcSucc(ScenarioFire<_type> *sf)					{ m_calcSucc = sf; }				// was introduced to deal with purging hidden time steps

protected:
	virtual void inspectPolygons(const PolysetOperation operation) override;
	virtual bool keepPolygon(XY_PolyLL_BaseTempl<_type>*poly, bool suggest_keeping, const PolysetOperation operation) const override;
};


template<class _type>
class ScenarioFireExport : public ScenarioFire<_type> {
protected:
	using XYPolyLLType = XY_PolyLL_BaseTempl<_type>;
	using XYPointType = XY_PointTempl<_type>;

public:
	using ScenarioFire<_type>::GetCacheScale;

public:
	DECLARE_OBJECT_CACHE_MT(ScenarioFireExport<_type>, ScenarioFireExport)

	ScenarioFireExport(const ScenarioTimeStep<_type> *timeStep, const IgnitionNode<_type> *ignition, ScenarioFire<_type> *pred) : ScenarioFire<_type>(timeStep, ignition, pred) { }

	virtual FireFrontExport<_type>*New() const override;
	virtual FireFrontExport<_type>*NewCopy(const XYPolyLLType &toCopy) const override;

	FireFrontExport<_type> *LH_Head() { return (FireFrontExport<_type>*)ScenarioFire<_type>::LH_Head(); }

	const class ScenarioExportRules *m_rules;
	std::uint16_t	m_flags;

protected:
	virtual bool exportPolygon(XYPolyLLType *externalPoly) const override;
	virtual bool associatePolygon(const XYPolyLLType *externalPoly, const XYPolyLLType *internalPoly) const override;
	virtual bool createExportFields(const TCHAR *driver_name, OGRLayerH layer) override;
	virtual bool setExportFields(const TCHAR *driver_name, OGRFeatureH feature, XYPolyLLType *poly) override;
};


template<class _type>
class XY_PolyLLPolyRef : public RefNode<XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>> {
public:
	__INLINE XY_PolyLLPolyRef* LN_Succ() const { return (XY_PolyLLPolyRef*)RefNode<XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>>::LN_Succ(); }
	__INLINE XY_PolyLLPolyRef* LN_Pred() const { return (XY_PolyLLPolyRef*)RefNode<XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>>::LN_Pred(); }

	DECLARE_OBJECT_CACHE_MT(XY_PolyLLPolyRef, XY_PolyLLPolyRef)
};


template<class _type>
class ScenarioTimeStep : public MinNode {
	using XYPointType = XY_PointTempl<_type>;
	using XYZPointType = XYZ_PointTempl<_type>;
	using XYRectangleType = XY_RectangleTempl<_type>;
	using XYPolySetType = XY_PolySetTempl<_type>;
	using XYPolyLLType = XY_PolyLL_BaseTempl<_type>;
	using XYPolyNodeType = XY_PolyLLNode<_type>;
	using XYPolyLLType2 = XY_PolyLL_Templ<XYPolyNodeType, _type>;
	using XYPolyType = XY_PolyTempl<_type>;
	using XYPolyConstType = XY_PolyConstTempl<_type>;

public:
	DECLARE_OBJECT_CACHE_MT(ScenarioTimeStep<_type>, ScenarioTimeStep)

	ScenarioTimeStep(Scenario<_type> *scenario, const WTime &event_end, bool simulation_end);
	virtual ~ScenarioTimeStep();

	__INLINE ScenarioTimeStep<_type> *LN_Succ() const	{ return (ScenarioTimeStep<_type>*)MinNode::LN_Succ(); };
	__INLINE ScenarioTimeStep<_type> *LN_Pred() const	{ return (ScenarioTimeStep<_type>*)MinNode::LN_Pred(); };

	CRWThreadSemaphore																m_lock;
	CThreadSemaphore																m_c_lock;			// only used for race conditions in calculating the centroid for the timestep's polygons
	std::uint64_t																	m_tickCountStart,
																					m_tickCountEnd;

	std::chrono::time_point<std::chrono::system_clock>								m_realtimeStart,
																					m_realtimeEnd;

	std::uint64_t																	m_memoryBegin, m_memoryEnd;

protected:
	XYPointType																		m_curr_ll, m_curr_ur;	// in UTM

private:
	XYPointType																		m_centroid;				// also in UTM

public:
	// a few helper routines matching what's in scenariocache, since we want to support the grid eventually growing
	const XYPointType& current_ll() const;
	const XYPointType& current_ur() const;

	std::uint64_t toGridScaleX(const XY_Point& loc) const;	// from UTM to grid coordinates (false origin, false scaling) - important for calculating some statistics
	std::uint64_t toGridScaleY(const XY_Point& loc) const;	// from UTM to grid coordinates (false origin, false scaling) - important for calculating some statistics

#ifdef USE_BIGFLOATS
	std::uint64_t toGridScaleX(const XYPointType& loc) const;	// from UTM to grid coordinates (false origin, false scaling) - important for calculating some statistics
	std::uint64_t toGridScaleY(const XYPointType& loc) const;	// from UTM to grid coordinates (false origin, false scaling) - important for calculating some statistics
#endif

	WTime																			m_time;			// the current time for this time step - this is the time at which this time
																									// step begins burning and is inclusive - it goes up to 1 second prior to LN_Succ()'s time
	Scenario<_type>																	*m_scenario;
																									// many times
	MinListTempl<ScenarioFire<_type>>												m_fires;
	RefList<ScenarioFire<_type>, ActiveFire<_type>>									m_activeFiresState;		// used to record the state of these active fires on a display timestep, so that we can easily do a step-back to a prior state
	std::vector<XY_PolyLLSetBB<_type>*>												*m_vectorBreaksLL;
	RefList<XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>, XY_PolyLLPolyRef<_type>>	m_staticVectorBreaksLL;
	std::uint32_t																	m_displayable : 1,		// if this is a displayable time step
																					m_evented : 1,			// if this time step ended on an event (if false, then it ended due to logic around ROS, etc.)
																				    m_ignitioned : 1;
	std::uint32_t																	m_assetCount;
	UnwindMetrics																	m_advanceMetrics,
																					m_setMetrics;
	StopConditionState																m_stopConditions;

	double MinimumROSRatio() const;
	double MaximumROS() const;
	double MaximumCardinalROS() const;
	std::uint32_t GetNumFireFronts() const;
	const FireFront<_type> *GetFireFront(std::uint32_t index);
	bool Centroid(XY_PointTempl<_type>* centroid);

	bool BoundingBox(XYRectangleType &bbox) const;
	std::int32_t PointInArea(const XYPointType &point) const;
	FirePoint<_type> *GetNearestPoint(const XYPointType &pt, bool all_points, FireFront<_type> **firefront, bool must_be_inside) const;

	void PreCalculation();
	void PostCalculation();

	bool AdvanceFires();							// copies fires from the previous step, "grows" them based on the length of
													// the time step from prev to this, and smooths automatically
	void SimplifyFires();								// here, we get rid of points that are too close to each other
	void SimplifyFiresNull();
	void UnWindFires(bool advance);					// removes knots from existing fires
	void TrackFires();								// ray traces looking for barriers
	void TrackFiresNull();
	void UnOverlapFires();							// this deals with fires contacting each other
	void UnOverlapFiresNull();
	bool AddIgnitions();							// adds newly started ignitions
	void AddFirePoints();							// adds new fire points to satisfy our desired sampling
	void StatsFires();								// calculates new FBP speeds, directions for each active fire vertex
	bool CheckAssets(bool& make_displayable);		// checks and computes arrival times for any of the asset associated with the scenario, returns whether the simulation is now done
	bool CheckStops(HRESULT &condition);			// checks if any conditions for early abort/stop of the simulation is present, returns whether the simulation is done

	std::uint32_t NumActivePoints() const;
	HRESULT RetrieveStat(const std::uint16_t stat, double *stats) const;
	HRESULT RetrieveStat(const std::uint16_t stat, PolymorphicAttribute* stats) const;
	HRESULT RetrieveStat(const std::uint16_t stat, const double greater_equal, const double less_than, double*stats) const;
	HRESULT RetrieveStat(const std::uint16_t stat, ICWFGM_Fuel* fuel, const std::uint16_t discretize, ScenarioTimeStep<_type>* sts_prev, double* stats) const;

	std::int32_t pointInsideVectors(const XYPointType& pt) const;

private:
	void addPoint(const XYPointType &loc, bool &valid, XYPolySetType &start);
	void addIgnition(IgnitionNode<_type> *ignition);
	void buildVectorBreaks();
	void reviewStaticBreaks();
	void advanceFire(ActiveFire<_type> *af);
	void retrieveDStat(std::uint16_t stat, ICWFGM_Fuel* fuel, std::uint16_t discretize, ScenarioTimeStep<_type>* sts_prev, double* stats, double step,
		const std::uint64_t cx, const std::uint64_t cy, boost::multi_array<typename Scenario<_type>::closest_calc, 2>& pt_array, CThreadSemaphore* add_lock) const;

	bool findFirstIgnitionCentroid(XY_PointTempl<_type>& centroid) const;

public:
	void RecordActiveFires();
	void RestoreActiveFires();

	bool canBurn() const;	// returns whether at least one of the fire can burn (or, not and operation)
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif

#endif
