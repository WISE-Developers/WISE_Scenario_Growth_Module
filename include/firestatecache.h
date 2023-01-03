/**
 * WISE_Scenario_Growth_Module: firestatecache.h
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

#ifndef __FIRESTATECACHE_H
#define __FIRESTATECACHE_H

#include "semaphore.h"
#include "Thread.h"
#include "poly.h"
#include "CWFGM_Scenario.h"
#include "valuecache_mt.h"
#include "CoordinateConverter.h"
#include <vector>

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif


template<class _type>
class FirePoint;
template<class _type>
class FireFront;
template<class _type>
class ScenarioTimeStep;

using namespace HSS_Time;

template<class _type>
class ScenarioGridCache {
protected:
	using XYPointType = XY_PointTempl<_type>;
	using XYRectangleType = XY_RectangleTempl<_type>;

protected:
	std::uint16_t			m_plot_X, m_plot_Y;

private:
	alignas(sizeof(_type))
	XYPointType				m_ll, m_ur;				// kept in UTM (original) coordinates
													// various values to deal with to/from resolution scalings
	_type					m_resolution, m_resolution2, m_resolution3, m_iresolution, m_iresolution2, m_iresolution3;
	_type					m_breachAdvance, m_minArea;

protected:
	CRWThreadSemaphore	m_cplock;

	std::uint32_t arrayIndex(std::uint16_t x, std::uint16_t y) const;
	std::uint32_t arrayIndex(const XYPointType &pt) const;

public:
	CCWFGM_Scenario		*m_scenario;				// pointer back to the managing parent / COM object
	_type				resolution() const;
	_type				resolution2() const;
	_type				iresolution() const;
	_type				iresolution2() const;
	_type				iresolution3() const;
	_type				breachAdvance() const;
	_type				minFireArea() const;

	const XYPointType& start_ll() const;
	const XYPointType& start_center() const;
	const XYPointType& start_ur() const;

public:											// bunch of routines to go to/from UTM and resolutions
	void toInternal(XY_Point& loc) const;			// from UTM to whatever is used in the grid (false or actual scaling)
	void fromInternal(XY_Point& loc) const;			// from whatever is used in the grid (false or actual scaling) to UTM

	std::uint64_t toGridScaleX(const XY_Point& loc) const;	// from UTM to grid coordinates (false origin, false scaling) - important for calculating some statistics
	std::uint64_t toGridScaleY(const XY_Point& loc) const;	// from UTM to grid coordinates (false origin, false scaling) - important for calculating some statistics

	void toInternal1D(double& value) const;			// from UTM to whatever is used in the grid (false or actual scaling)
	void toInternal2D(double& value) const;			// from UTM to whatever is used in the grid (false or actual scaling)
	void toInternal3D(double& value) const;			// from UTM to whatever is used in the grid (false or actual scaling)
	void gridToInternal1D(double& value) const;		// from grid units (sometimes ideal for describing some values like spatialThreshold, perimeterResolution) to whatever is used in the grid (false or actual scaling)
	void gridToInternal2D(double& value) const;		// from grid units (sometimes ideal for describing some values like spatialThreshold, perimeterResolution) to whatever is used in the grid (false or actual scaling)

	void fromInternal1D(double& value) const;		// from whatever is used in the grid (false or actual scaling) to UTM
	void fromInternal2D(double& value) const;		// from whatever is used in the grid (false or actual scaling) to UTM
	void fromInternal3D(double& value) const;		// from whatever is used in the grid (false or actual scaling) to UTM
	void gridFromInternal1D(double& value) const;	// from whatever is used in the grid (false or actual scaling) to grid units (sometimes ideal for describing some values like spatialThreshold, perimeterResolution)
	void gridFromInternal2D(double& value) const;	// from whatever is used in the grid (false or actual scaling) to grid units (sometimes ideal for describing some values like spatialThreshold, perimeterResolution)

#ifdef USE_BIGFLOATS
	void toInternal(XYPointType& loc) const;
	void fromInternal(XYPointType& loc) const;

	std::uint64_t toGridScaleX(const XYPointType& loc) const;	// from UTM to grid coordinates (false origin, false scaling) - important for calculating some statistics
	std::uint64_t toGridScaleY(const XYPointType& loc) const;	// from UTM to grid coordinates (false origin, false scaling) - important for calculating some statistics

	void toInternal1D(_type& value) const;			// from UTM to whatever is used in the grid (false or actual scaling)
	void toInternal2D(_type& value) const;			// from UTM to whatever is used in the grid (false or actual scaling)
	void toInternal3D(_type& value) const;			// from UTM to whatever is used in the grid (false or actual scaling)
	void gridToInternal1D(_type& value) const;		// from grid units (sometimes ideal for describing some values like spatialThreshold, perimeterResolution) to whatever is used in the grid (false or actual scaling)
	void gridToInternal2D(_type& value) const;		// from grid units (sometimes ideal for describing some values like spatialThreshold, perimeterResolution) to whatever is used in the grid (false or actual scaling)

	void fromInternal1D(_type& value) const;		// from whatever is used in the grid (false or actual scaling) to UTM
	void fromInternal2D(_type& value) const;		// from whatever is used in the grid (false or actual scaling) to UTM
	void fromInternal3D(_type& value) const;		// from whatever is used in the grid (false or actual scaling) to UTM
	void gridFromInternal1D(_type& value) const;	// from whatever is used in the grid (false or actual scaling) to grid units (sometimes ideal for describing some values like spatialThreshold, perimeterResolution)
	void gridFromInternal2D(_type& value) const;	// from whatever is used in the grid (false or actual scaling) to grid units (sometimes ideal for describing some values like spatialThreshold, perimeterResolution)
#endif

public:
	ScenarioGridCache(CCWFGM_Scenario *scenario, const XY_Point &start_ll, const XY_Point &start_ur, const _type resolution /*, const std::uint16_t x_dim, const std::uint16_t y_dim*/);
	~ScenarioGridCache();

	void Size(const XYPointType &ll, const XYPointType &ur);
	void RecordTimeStep(ScenarioTimeStep<_type> *sts);
	bool RetrieveCPoint(const XYPointType &pt, const WTime &time, bool displayable, FirePoint<_type> **fp, FireFront<_type> **ff);
	bool RetrieveCPoint(const XYPointType& pt, const WTime &mintime, const WTime& time, bool displayable, FirePoint<_type>** fp, FireFront<_type>** ff);

protected:
	bool allocCPArray();

private:
	void grid_create(const XYPointType &ll, const XYPointType &ur);
	void grid_resize(const XYPointType &_ll, const XYPointType &_ur);
};


template<class _type>
class XY_PolyLLTimed : public XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type> {
public:
	WTime m_usedTime;

	XY_PolyLLTimed() : XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::XY_PolyLL_Templ(), m_usedTime(0, nullptr) {};
	XY_PolyLLTimed(const _type* xy_pairs, std::uint32_t array_size) : XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::XY_PolyLL_Templ(xy_pairs, array_size), m_usedTime(0, nullptr) {};
	XY_PolyLLTimed(const XY_PolyConstTempl<_type>& toCopy) : XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::XY_PolyLL_Templ(toCopy), m_usedTime(0, nullptr) {};
	XY_PolyLLTimed(const XY_PolyLL_BaseTempl<_type>& toCopy) : XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::XY_PolyLL_Templ(toCopy), m_usedTime(0, nullptr) {};
	XY_PolyLLTimed(const XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>& toCopy) : XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::XY_PolyLL_Templ(toCopy), m_usedTime(0, nullptr) {};
	XY_PolyLLTimed(XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>&& toMove) : XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::XY_PolyLL_Templ(toMove), m_usedTime(0, nullptr) {};
	XY_PolyLLTimed(const XY_PointTempl<_type>* pt_array, std::uint32_t array_size) : XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::XY_PolyLL_Templ(pt_array, array_size), m_usedTime(0, nullptr) {};

	__INLINE XY_PolyLLTimed* LN_Succ() const { return (XY_PolyLLTimed*)XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::LN_Succ(); }
	__INLINE XY_PolyLLTimed* LN_Pred() const { return (XY_PolyLLTimed*)XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>::LN_Pred(); }

	virtual bool Clip_Participates(APTR parm) const override {
		if (!parm)
			return true;
		WTime* time = (WTime*)parm;
		return Participates(*time);
	}

	bool Participates(const WTime& time) const {
		if (!m_usedTime.GetTotalMicroSeconds())
			return false;
		return (m_usedTime <= time);
	}
};


template<class _type>
class XY_PolyLLSetBB : public XY_PolyLL_Set<XY_PolyLLTimed<_type>, _type> {
public:
	XY_RectangleTempl<_type> box;
};


template<class _type>
class ScenarioCache : public ScenarioGridCache<_type> {

protected:
	using XYRectangleType = typename ScenarioGridCache<_type>::XYRectangleType;
	using XYZPointType = XYZ_PointTempl<_type>;
	using XYPointType = typename ScenarioGridCache<_type>::XYPointType;
	using XYPolyNodeType = XY_PolyLLNode<_type>;
	using XYPolyLLType = XY_PolyLL_Templ<XYPolyNodeType, _type>;
	using XYPolyConstType = XY_PolyConstTempl<_type>;

private:
	using ScenarioGridCache<_type>::allocCPArray;
	using ScenarioGridCache<_type>::m_plot_X;
	using ScenarioGridCache<_type>::m_plot_Y;

public:
	using ScenarioGridCache<_type>::m_scenario;
	using ScenarioGridCache<_type>::Size;
	using ScenarioGridCache<_type>::toInternal;
	using ScenarioGridCache<_type>::fromInternal;
	using ScenarioGridCache<_type>::resolution;
	using ScenarioGridCache<_type>::resolution2;
	using ScenarioGridCache<_type>::start_ll;
	using ScenarioGridCache<_type>::start_ur;

public:
	ScenarioCache(CCWFGM_Scenario *scenario, const XY_Point &start_ll, const XY_Point& start_ur, const _type resolution, const double landscapeFMC, const double landscapeElev, std::uint32_t numthreads);
	~ScenarioCache();

	const _type		m_specifiedFMC_Landscape;
	const _type		m_specifiedElev_Landscape;

	bool			m_multithread, m_assets;
	std::uint32_t	m_numthreads;

	CWorkerThreadPool	*m_pool;		// for multi-CPU operations

	bool AllocCPArray();

	__INLINE std::uint32_t						StaticVectorBreakCount() const					{ return (std::uint32_t)m_staticVectorBreaksLL->size(); }
	std::uint32_t								AssetCount() const;
	__INLINE bool								StaticVectorBreak() const						{ return (m_staticVectorBreaksLL) ? true : false; };
	__INLINE const XY_PolyLLSetBB<_type>		*StaticVectorBreak(std::uint32_t index) const	{ return (*m_staticVectorBreaksLL)[index]; };

	bool IsNonFuel(const WTime &time, const XYPointType &pt, bool &valid) const;
	bool IsNonFuelUTM(const WTime& time, const XYPointType& pt, bool& valid) const;
	bool IsNonFuel_NotCached(const WTime &time, const XYPointType &pt, bool &valid, XYRectangleType *cache_bbox) const;

	ICWFGM_Fuel *GetFuel(const WTime &time, const XYPointType &pt, bool &valid) const;

#if defined(SUPPORT_BIGFLOATS) && defined(USE_BIGFLOATS)
	ICWFGM_Fuel* GetFuel(const WTime& time, const XY_Point& pt, bool& valid) const;
#endif
	ICWFGM_Fuel* GetFuel(const WTime& time, const XYZPointType& pt, bool& valid) const;
	ICWFGM_Fuel* GetFuel_NotCached(const WTime& time, const XYPointType& pt, bool& valid) const;
	ICWFGM_Fuel* GetFuelUTM_NotCached(const WTime& time, const XYPointType& pt, bool& valid) const;

	void GetCorrectedFuel(const XYZPointType &c_pt, const WTime &time, ICWFGM_Fuel *fuel, CCWFGM_FuelOverrides &overrides);
	void GetCorrectedFuel(const XYPointType &pt, const WTime& time, ICWFGM_Fuel* fuel, CCWFGM_FuelOverrides& overrides);
	void GetCorrectedFuelUTM(const XYPointType &_pt, const WTime& time, ICWFGM_Fuel* fuel, CCWFGM_FuelOverrides& overrides);

							// done to deal with PDF, PC, % cure grass, CBH grid layers
	CCoordinateConverter	m_coordinateConverter;
	void IgnitionExtents(XYRectangleType &bbox);

protected:
	void buildStaticVectorBreaks();
	void buildAssets();
	std::vector<XY_PolyLLSetBB<_type>*>		*m_staticVectorBreaksLL;

	void PreCalculation();
	void PostCalculation();

public:
	void InitThreadPool(bool multithread);

	bool CanBurn(const WTime &datetime, const XYPointType &centroid, const XYPointType &pt, const double rh, const double WindSpeed, const double fwi, const double isi);
	bool CanBurn(const WTime &datetime, const XYPointType& centroid);
	bool CanBurnTime(const WTime &dateTime, const XYPointType &centroid, WTimeSpan &start, WTimeSpan &end);

private:
	bool isNonFuelUTM_NotCached(const WTime& time, const XYPointType& _pt, bool& valid, XYRectangleType* cache_bbox) const;

	template<class T>
	friend bool cacheRetrieve(typename ScenarioCache<T>::fuel_cache_key *entry, APTR lookup);
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif

#endif
