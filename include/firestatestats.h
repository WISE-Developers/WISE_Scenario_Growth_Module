/**
 * WISE_Scenario_Growth_Module: firestatestats.h
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

#ifndef __FIRESTATESTATS_H
#define __FIRESTATESTATS_H

#include "firepoint.h"

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif

template<class _type>
class ScenarioFire;

template<class _type>
class FireFrontStats : public XY_PolyLL_Templ<FirePoint<_type>, _type> {
protected:
	using XYPointType = XY_PointTempl<_type>;
	using XYZPointType = XYZ_PointTempl<_type>;
	using XYPolyConstType = XY_PolyConstTempl<_type>;
	using XYPolyLLType = XY_PolyLL_BaseTempl<_type>;

public:
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::LH_Head;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::LH_Tail;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::EnableCaching;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::Length;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::Area;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::NumPoints;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::ClosestPoint;
	using XY_PolyLL_Templ<FirePoint<_type>, _type>::SetPoint;

protected:
	const ScenarioFire<_type>* m_fire;			// moved to here for MaximumBurnDistance(), but initialized in the derived class's constructor - set in
												// the constructor for FireFront, which inherits from this class
public:
	FireFrontStats() : XY_PolyLL_Templ<FirePoint<_type>, _type>()																{ m_cachedArea = 0.0; };
	FireFrontStats(const XYPolyConstType &toCopy) : XY_PolyLL_Templ<FirePoint<_type>, _type>(toCopy)							{ m_cachedArea = 0.0; };
	FireFrontStats(const XY_PolyLL_Templ<FirePoint<_type>, _type> & toCopy) : XY_PolyLL_Templ<FirePoint<_type>, _type>(toCopy)		{ m_cachedArea = 0.0; };

	double ActivePerimeter() const;
	double ExteriorPerimeter() const;
	double TotalPerimeter() const;
	double PreviousMinimumROSRatio(const bool early_abort) const;
	double MinimumROSRatio() const;
	double MinimumROS() const;
	double MaximumROS() const;
	double MaximumCardinalROS() const;
	double AverageROS() const;

	double MaximumFI() const;
	double MaximumHFI() const;
	double MaximumCFB() const;
	double MaximumHCFB() const;
	double MaximumCFC() const;
	double MaximumSFC() const;
	double MaximumTFC() const;
	double MaximumFlameLength() const;
	virtual double MaximumBurnDistance() const;

	std::uint32_t NumActivePoints() const;

	HRESULT RetrieveStat(const std::uint16_t stat, double*stats) const;
	HRESULT RetrieveStat(const std::uint16_t stat, const double greater_equal, const double less_than, double*stats) const;

protected:
	_type _area() const override;
	void postClearCache() override																				{ m_cachedArea = 0.0; };
	void postEnableCache(bool /*cache_active*/) override														{ m_cachedArea = 0.0; };
	void postRescanRanges(bool /*force*/) const override														{ const_cast<FireFrontStats<_type>*>(this)->m_cachedArea = 0.0; };
	void postInsertPoint(const XY_PolyLLNode<_type>* /*point*/) override										{ m_cachedArea = 0.0; };
	void preRemovePoint(const XY_PolyLLNode<_type>* /*point*/) override											{ m_cachedArea = 0.0; };
	void preSetPoint(const XY_PolyLLNode<_type>* /*point*/, const XYPointType & /*newPt*/) override				{ m_cachedArea = 0.0; };
	void postReverseRotation() override																			{ m_cachedArea = 0.0; };

	double maximumBurnDistance(const ScenarioFire<_type>* fire) const;

private:
	_type m_cachedArea;
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif

#endif
