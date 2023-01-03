/**
 * WISE_Scenario_Growth_Module: ScenarioAsset.h
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

#if !defined(AFX_SCENARIOASSET_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_)
#define AFX_SCENARIOASSET_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "WTime.h"
#include "FireEngine.h"
#include "poly.h"
#include "objectcache_mt.h"
#include "firepoint.h"
#include "ScenarioExportRules.h"
#include <boost/intrusive_ptr.hpp>
#include "ICWFGM_Asset.h"

using namespace HSS_Time;

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif

template<class _type>
class FireFront;
template<class _type>
class ActiveFire;


template<class _type>
class AssetGeometryNode : public MinNode {
protected:
	using XYPolyLLBaseType = XY_PolyLL_BaseTempl<_type>;
	using XYPolyNodeType = XY_PolyLLNode<_type>;
	using XYPolyLLType = XY_PolyLL_Templ<XYPolyNodeType, _type>;

public:
	AssetGeometryNode(WTimeManager* timeManager) : m_arrivalTime((ULONGLONG)0, timeManager) { m_arrived = false; m_closestFirePoint = nullptr; m_closestFireFront = nullptr; }
	AssetGeometryNode* LN_Succ() const { return (AssetGeometryNode*)MinNode::LN_Succ(); };
	AssetGeometryNode* LN_Pred() const { return (AssetGeometryNode*)MinNode::LN_Pred(); };

	XYPolyLLType						m_geometry;			// one of the geometry's from m_asset below, look at member variable to indicate point, line polygon
	WTime								m_arrivalTime;		// when the fire reaches this geomtry
	bool								m_arrived;			// whether m_arrivalTime is valid
															// for finding the critical path, this is valid for intersecting a point asset, it becomes a lot harder if
															// it's a polygon or polyline asset
	FirePoint<_type>					m_closestPoint;		// we record the closest point as X,Y so that we have that location, even if interim timesteps are deleted,
															// if m_closestFirePoint is NULL, then consider this value invalid
	FirePoint<_type>					*m_closestFirePoint;// pointer to the closest point - may change if Purge() is called, to be the last point in a prior display
															// timestep, we can track history using FirePoint's m_prevPoint, m_succPoint
	FireFront<_type>					*m_closestFireFront;// firefront holding the closest FirePoint

	void fixClosestPoint();
	void BuildCriticalPath(WTimeManager* manager, class CriticalPath& set, const ScenarioExportRules* rules) const;

protected:
	FireFront<_type>* findFireFront(const class ScenarioTimeStep<_type>* sts, const class ActiveFire<_type>* af, const FirePoint<_type>* point) const;
	void constructAttributes(const ScenarioExportRules* rules, const FirePoint<_type>* point, const WTime& time, std::vector<GDAL_Attribute>* attributes) const;

public:

	DECLARE_OBJECT_CACHE_MT(AssetGeometryNode<_type>, AssetGeometryNode)
};

template<class _type>
class AssetNode : public MinNode {
    public:
	AssetNode();
	~AssetNode();
	AssetNode* LN_Succ() const { return (AssetNode*)MinNode::LN_Succ(); };
	AssetNode* LN_Pred() const { return (AssetNode*)MinNode::LN_Pred(); };

	boost::intrusive_ptr<ICWFGM_Asset>	m_asset;			// original COM object
	std::uint32_t						m_operation;		// possible values include:
															// -2 means no effect - simulation runs to completion
															// -1 means each asset geometries have been reached
															// 0 means no effect - simulation runs to completion - default
															// > 0 is how many asset geometries must be reached before the simulation is stopped (simple count)
	MinListTempl<AssetGeometryNode<_type>>		m_geometry;	// local copy (for performance) of computed data from m_asset

	DECLARE_OBJECT_CACHE_MT(AssetNode<_type>, AssetNode)
};					

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif

#endif // !defined(AFX_SCENARIOIGNITION_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_)
