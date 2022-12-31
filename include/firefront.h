/**
 * WISE_Scenario_Growth_Module: firefront.h
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

#ifndef __FIREFRONT_H
#define __FIREFRONT_H

#include <type_traits>

#include "firestatestats.h"
#include "ScenarioIgnition.h"


#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 4)
#endif


template<class _type>
class ScenarioFire;
template<class _type>
struct FireStateCallback;
template<class _type>
struct stepVoxelStart;

template<class _type>
class FireFront : public FireFrontStats<_type> {
public:
	using XYZPointType = typename FireFrontStats<_type>::XYZPointType;
	using XYPolyConstType = typename FireFrontStats<_type>::XYPolyConstType;
	using XYPolyLLType = typename FireFrontStats<_type>::XYPolyLLType;
	using XYPointType = typename FireFrontStats<_type>::XYPointType;
	using XYPolyNodeType = XY_PolyLLNode<_type>;
	using XYPolyRefType = XY_PolyLLNodeRef<_type>;
	using XYPolyDistType = XY_PolyLLDistRef<_type>;
	using XYLineType = XY_LineTempl<_type>;
	using XYRectangleType = XY_RectangleTempl<_type>;

	friend ScenarioFire<_type>;
	friend std::uint32_t AFX_CDECL stepVoxelInit(APTR parameter);
	friend std::uint32_t AFX_CDECL advancePointInit(APTR parameter);
	friend std::uint32_t AFX_CDECL stepVectorInit(APTR parameter);
	template<class T>
	friend int stepVoxelCallback(APTR parameter, const  XYZ_PointTempl<T> *entry, const  XYZ_PointTempl<T> *exit);

public:
	using FireFrontStats<_type>::m_fire;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::m_ptList;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::EnableCaching;
	using XY_PolyLL_BaseTempl<_type>::NumPoints;
	using XY_PolyLL_BaseTempl<_type>::RemovePoint;
	using XY_PolyLL_BaseTempl<_type>::InsertPoint;
	using XY_PolyLL_BaseTempl<_type>::GetCacheScale;
	using FireFrontStats<_type>::PreviousMinimumROSRatio;
	using FireFrontStats<_type>::ClosestPoint;
	using FireFrontStats<_type>::SetPoint;

public:
	DECLARE_OBJECT_CACHE_MT(FireFront<_type>, FireFront)

	FireFront();							// this constructor is actually never called - it's just here to allow the template class to compile
	FireFront(const XYPolyConstType &ff);
	FireFront(const XYPolyLLType &ff);				// this constructor is actually never called - it's just here to allow the template class to compile
	FireFront(const ScenarioFire<_type> *fire);
	FireFront(const ScenarioFire<_type> *fire, const XYPolyConstType &toCopy);
	FireFront(const ScenarioFire<_type> *fire, const FireFront<_type> &toCopy);
	virtual ~FireFront() = default;

	__INLINE FireFront<_type> *LN_Succ() const				{ return (FireFront<_type>*)FireFrontStats<_type>::LN_Succ(); };
	__INLINE FireFront<_type> *LN_Pred() const				{ return (FireFront<_type>*)FireFrontStats<_type>::LN_Pred(); };

	__INLINE FirePoint<_type> *LH_Head() const				{ return static_cast<FirePoint<_type>*>(XY_PolyLL_BaseTempl<_type>::LH_Head()); }
	__INLINE FirePoint<_type> *LH_Tail() const				{ return static_cast<FirePoint<_type>*>(XY_PolyLL_BaseTempl<_type>::LH_Tail()); }

	virtual FirePoint<_type> *New() const override;
	virtual const XYPolyNodeType *ChooseToKeep(const XYPolyNodeType *first, const XYPolyNodeType *second) const override;

	__INLINE const ScenarioFire<_type> *Fire() const			{ return FireFrontStats<_type>::m_fire; };

	FirePoint<_type> *GetNearestPoint(const XYPointType &pt, bool all_points);

	bool AdvanceFire(const _type scale);
	std::uint32_t Simplify();
	void TrackFireGrid();
	void TrackFireVector();
	void AddPoints();
	void GrowPoints();

protected:
	virtual bool FindPoint_Participates(const XYPolyNodeType *point, APTR parm) const override;

private:
	void CopyPoints(const XYPolyLLType &toCopy, std::uint32_t status);
	bool advancePoint(const _type scale, const FirePoint<_type> *prev, FirePoint<_type> *curr, const FirePoint<_type> *next,
	    const XYPointType &prev_e_ros, const XYPointType &curr_e_ros, const XYPointType &next_e_ros);
	void equiDistantPoints(const FirePoint<_type> *start, const FirePoint<_type> *end, _type dist_factor);

	int stepVoxel(APTR parameter, const XYZPointType *entry, const XYZPointType *exit);
	void trackPointGrid(const FirePoint<_type> *actual_fp, FirePoint<_type> *fp, stepVoxelStart<_type> *svs);
	void trackPointPrevFire(const FirePoint<_type> *actual_fp, FirePoint<_type> *fp, stepVoxelStart<_type> *svs);
	void trackPointVector(const FirePoint<_type> *actual_fp, FirePoint<_type> *fp, stepVoxelStart<_type> *svs);
	std::int32_t pointInsideVectors(const XYPointType &pt) const;

	bool simplify(FirePoint<_type> *prev, FirePoint<_type> *curr, FirePoint<_type> *succ, const _type perimeterSpacing, const _type perimeterResolution,
		const bool convex, const _type n_pr, const _type s_pr, FirePoint<_type> **sel, _type*sel_dist, _type*sel_angle);
	_type adjustAngle(bool convex, _type angle);
};


template<class _type>
class FireFrontExport : public FireFront<_type> {
public:
	using FireFrontStats<_type>::maximumBurnDistance;

public:
	DECLARE_OBJECT_CACHE_MT(FireFrontExport<_type>, FireFrontExport)

	FireFrontExport() : FireFront<_type>(), m_time((std::uint64_t)0, nullptr), m_assetTime((std::uint64_t)0, false), m_assetCount(0) { m_origScenarioFire = nullptr; m_origArea = m_origPerimeter = m_origExteriorPerimeter = m_origActivePerimeter = m_origDistance = -1.0; }
	FireFrontExport(const ScenarioFire<_type> *fire) : FireFront<_type>(fire), m_time((std::uint64_t)0, nullptr), m_assetTime((std::uint64_t)0, false), m_assetCount(0)
									{ m_origScenarioFire = nullptr; m_origArea = m_origPerimeter = m_origExteriorPerimeter = m_origActivePerimeter = m_origDistance = -1.0; }
	FireFrontExport(const ScenarioFire<_type> *fire, const FireFront<_type> &toCopy) : FireFront<_type>(fire, toCopy), m_time((std::uint64_t)0, nullptr), m_assetTime((std::uint64_t)0, false), m_assetCount(0)
									{ m_origScenarioFire = nullptr; m_origArea = m_origPerimeter = m_origExteriorPerimeter = m_origActivePerimeter = m_origDistance = -1.0; }

	__INLINE FireFrontExport<_type> *LN_Succ() const			{ return (FireFrontExport<_type>*)FireFront<_type>::LN_Succ(); };
	__INLINE FireFrontExport<_type> *LN_Pred() const			{ return (FireFrontExport<_type>*)FireFront<_type>::LN_Pred(); };

	virtual double MaximumBurnDistance() const;

	ScenarioFire<_type> *m_origScenarioFire;
	WTime			m_time;										// copy from the ScenarioTimeStep object from which this was copied
	WTimeSpan		m_assetTime;
	std::uint32_t	m_assetCount;
	_type			m_origArea, m_origPerimeter, m_origExteriorPerimeter, m_origActivePerimeter, m_origDistance;
};


#include "firestatecache.h"


template<class _type>
struct growVoxelParms {
	FireFront<_type> *self;
	ScenarioFire<_type> *self_fire;
	const ScenarioTimeStep<_type> *self_fire_timestep;
	XY_PointTempl<_type> centroid;
	ICWFGM_GridEngine *grid;
	WTimeSpan accel_dtime, day_portion;
	double latitude, longitude;
	double m_specifiedFMC, m_specifiedFMC_Landscape;
	double m_defaultElevation, m_specifiedElev_Landscape;
	double m_dwd, m_owd;
	ICWFGM_Target* target;
	std::uint32_t target_idx, target_sub_idx;
	std::uint32_t queue_up;
};


template<class _type>
struct growVoxelIterator {
	ScenarioFire<_type>* VOLATILE sf;
	FireFront<_type>* VOLATILE ff;
	FirePoint<_type>* VOLATILE fp;
	CThreadSemaphore lock_ll;
	growVoxelParms<_type> gvp;
};

#endif

template<class _type>
struct growPointStruct {
	FirePoint<_type> *fp;
	struct growVoxelParms<_type> *gvs;
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif

#endif
