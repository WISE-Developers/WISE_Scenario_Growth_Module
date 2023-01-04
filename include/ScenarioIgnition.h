/**
 * WISE_Scenario_Growth_Module: ScenarioIgnition.h
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

#if !defined(AFX_SCENARIOIGNITION_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_)
#define AFX_SCENARIOIGNITION_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "WTime.h"
#include "CWFGM_Fire.h"
#include "linklist.h"
#include "rectangles.h"
#include "objectcache_mt.h"

using namespace HSS_Time;

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 2)
#endif

template<class _type>
class IgnitionNode : public MinNode {
    public:
	IgnitionNode(WTimeManager *tm);
	IgnitionNode(const IgnitionNode<_type> &toCopy, WTimeManager *tm);
	~IgnitionNode() = default;

	IgnitionNode<_type> *LN_Succ() const			{ return (IgnitionNode<_type>*)MinNode::LN_Succ(); };
	IgnitionNode<_type> *LN_Pred() const			{ return (IgnitionNode<_type>*)MinNode::LN_Pred(); };

	void Reset(const WTime& startTimeOverride, const WTimeSpan &dt);

	boost::intrusive_ptr<CCWFGM_Ignition>	m_ignitionCOM;				// handle to an ignition source
	WTime									m_ignitionTime;				// when the ignition is to take place
	bool									m_ignitionHasPoint,			// whether or not there is a point ignition in this set
											m_ignitionHasPoly;			// whether or not there is a polygon ignition in this set

	bool getPoint(_type dx, _type dy, XY_PointTempl<_type>& loc) const;

	DECLARE_OBJECT_CACHE_MT(IgnitionNode<_type>, IgnitionNode)

};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif

#endif // !defined(AFX_SCENARIOIGNITION_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_)
