/**
 * WISE_Scenario_Growth_Module: firestatestats.cpp
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
#include "CWFGM_Scenario.h"
#include "firestatestats.h"
#include "FireEngine_ext.h"
#include "results.h"
#include "ScenarioTimeStep.h"
#include "scenario.h"


template<class _type>
std::uint32_t FireFrontStats<_type>::NumActivePoints() const {
	std::uint32_t cnt = 0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if (!fp->m_status)
			cnt++;
		fp = fp->LN_Succ();
	}
	return cnt;
}


template<class _type>
double FireFrontStats<_type>::ActivePerimeter() const {		// this is still different from Perimeter() because although a fire front won't include
								// points which are interior to the fire, it can still include points which are stopped due
								// to contact with a boundary, etc.  This only reports edges which contain a point which still burns
	double p = 0.0;
	FirePoint<_type>	*fp = LH_Head(),
						*pred = LH_Tail();
	while (fp->LN_Succ()) {
		if ((!fp->m_status) || (!pred->m_status))
			p += fp->DistanceTo(*pred);
		pred = fp;
		fp = fp->LN_Succ();
	}
	return p;
}


template<class _type>
_type FireFrontStats<_type>::_area() const {
	if ((EnableCaching()) && (m_cachedArea != 0.0)) {

    #ifdef _DEBUG
		_type area = XY_PolyLL_Templ<FirePoint<_type>, _type>::_area();
		weak_assert(area == m_cachedArea);
    #endif

		return m_cachedArea;
	}

	const_cast<FireFrontStats<_type>*>(this)->m_cachedArea = XY_PolyLL_Templ<FirePoint<_type>, _type>::_area();
	return m_cachedArea;
}


template<class _type>
double FireFrontStats<_type>::ExteriorPerimeter() const {		// this is still different from Perimeter() because although a fire front won't include
								// points which are interior to the fire, it can still include points which are stopped due
								// to contact with a boundary, etc.  This stat still includes enclaves.
	double p = 0.0;
	FirePoint<_type>	*fp = LH_Head(),
						*pred = LH_Tail();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_FIRE) || (pred->m_status != FP_FLAG_FIRE))
			p += fp->DistanceTo(*pred);
    #ifdef _DEBUG
		else {
			;
		}
    #endif
		pred = fp;
		fp = fp->LN_Succ();
	}
	return p;
}


template<class _type>
double FireFrontStats<_type>::TotalPerimeter() const {
	weak_assert(IsPolygon());
	return (double)Length();
}


template<class _type>
double FireFrontStats<_type>::AverageROS() const {			// RWB: 010426: this is only needed to be called for the very first iteration to get a perimeter growth rate
								// since we start with a perfectly round circle of a size that doeesn't relate back a reasonable size in meters - just in
								// the grid - it will squew the very first calculation for perimeter growth rate to a meaningless number.  But, we still
	double p = 0.0, c = 0.0, dist;				// have an ROS for that, so we can "fake" a perimeter growth rate for this very first step to something that likely isn't
	FirePoint<_type> *fp = LH_Head(),			// useful, but certainly isn't way out in left field for possible values.  Since we know that on the first iteration, the
					 *pred = LH_Tail();			// fire is a round circle with evenly spaced points, this calculation becomes really, really easy.
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL)) {
			p += fp->m_vector_ros;
			dist = fp->DistanceTo(*pred) + fp->DistanceTo(*fp->LN_SuccWrap());
			c += dist;
			
		}
		pred = fp;
		fp = fp->LN_Succ();
	}
	return p / c * 0.5;

}


template<class _type>
double FireFrontStats<_type>::MaximumROS() const {
	double _max = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_vector_ros > _max))
			_max = fp->m_vector_ros;
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MaximumCardinalROS() const {
	double _max = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL)) {
			double cardinalROS;
			_type sn, cs;
			::sincos(fp->m_fbp_raz, &sn, &cs);
			cardinalROS = max(sn, cs) * fp->m_vector_ros;
			if (cardinalROS > _max)
				_max = fp->m_vector_ros;
		}
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MinimumROS() const {
	double _min = 1000.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_vector_ros < _min))
			_min = fp->m_vector_ros;
		fp = fp->LN_Succ();
	}
	return _min;

}


template<class _type>
double FireFrontStats<_type>::PreviousMinimumROSRatio(const bool early_abort) const {
															// there's a choice of 2 implementations - this one, which simply looks at each point's prior point to make a decision,
															// or find a prior point and walk along that fire perimeter.  I chose the former because the untangler may do other poly
															// joins and splits and we want to stick with what we have associated, rather than introduce some heuristic to look for
															// a specific set of points from the prior time step
	bool found = false;
	double _min = 1.0;
	FirePoint<_type>* fp = LH_Head();
	while (fp->LN_Succ()) {
		if (fp->m_prevPoint) {
			if ((fp->m_prevPoint->m_status == 0) && (fp->m_prevPoint->m_fbp_ros_ratio < _min)) {
				_min = fp->m_prevPoint->m_fbp_ros_ratio;
				found = true;
				if ((early_abort) && (_min < 0.90))
					return _min;							// quick abort since we've already found 
			}
		}
		fp = fp->LN_Succ();
	}
	if (found)
		return _min;
	return 0.0;
}


template<class _type>
double FireFrontStats<_type>::MinimumROSRatio() const {
	double _min = 1.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status == 0) && (fp->m_fbp_ros_ratio < _min)) {
			_min = fp->m_fbp_ros_ratio;
		}
		fp = fp->LN_Succ();
	}
	return _min;
}


template<class _type>
double FireFrontStats<_type>::MaximumCFB() const {
	double _max = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_vector_cfb > _max)) 
			_max = fp->m_vector_cfb;
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MaximumHCFB() const {
	double _max = 0.0;
	FirePoint<_type>* fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_fbp_cfb > _max))
			_max = fp->m_fbp_cfb;
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MaximumCFC() const {
	double _max = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_vector_cfc > _max))
			_max = fp->m_vector_cfc;
		fp = fp->LN_Succ();
	}
	return _max;
}



template<class _type>
double FireFrontStats<_type>::MaximumTFC() const {
	double _max = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_vector_tfc > _max)) 
			_max = fp->m_vector_tfc;
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MaximumSFC() const {
	double _max = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_vector_sfc > _max))
			_max = fp->m_vector_sfc;
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MaximumFI() const {
	double _max = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_vector_fi > _max)) 
			_max = fp->m_vector_fi;
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MaximumHFI() const {
	double _max = 0.0;
	FirePoint<_type>* fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_fbp_fi > _max))
			_max = fp->m_fbp_fi;
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MaximumFlameLength() const {
	double _max = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if ((fp->m_status != FP_FLAG_NO_ROS) && (fp->m_status != FP_FLAG_NOFUEL) && (fp->m_flameLength > _max)) 
			_max = fp->m_flameLength;
		fp = fp->LN_Succ();
	}
	return _max;
}


template<class _type>
double FireFrontStats<_type>::MaximumBurnDistance() const {
	return maximumBurnDistance(m_fire);
}


template<class _type>
double FireFrontExport<_type>::MaximumBurnDistance() const {
	return maximumBurnDistance(m_origScenarioFire);
}


template<class _type>
double FireFrontStats<_type>::maximumBurnDistance(const ScenarioFire<_type>* fire) const {
	_type _max = 0.0;
	XYPointType loc;

	if (fire) {
		if (fire->Ignition()->getPoint(fire->TimeStep()->m_scenario->m_scenario->m_dx,
			fire->TimeStep()->m_scenario->m_scenario->m_dy,
			loc)) {
			fire->TimeStep()->m_scenario->toInternal(loc);
			((FireFrontStats<_type>*)this)->FurthestPoint(loc, nullptr, &_max);
		}
	}
	return (double)_max;
}


template<class _type>
HRESULT FireFrontStats<_type>::RetrieveStat(const std::uint16_t stat, double *stats) const {
	switch (stat) {
		case CWFGM_FIRE_STAT_AREA:					*stats = (double)Area();				return S_OK;
		case CWFGM_FIRE_STAT_NUM_POINTS:			*stats = (double)NumPoints();			return S_OK;
		case CWFGM_FIRE_STAT_NUM_ACTIVE_POINTS:		*stats = (double)NumActivePoints();		return S_OK;
		case CWFGM_FIRE_STAT_ACTIVE_PERIMETER:		*stats = (double)ActivePerimeter();		return S_OK;
		case CWFGM_FIRE_STAT_EXTERIOR_PERIMETER:	*stats = (double)ExteriorPerimeter();	return S_OK;
		case CWFGM_FIRE_STAT_TOTAL_PERIMETER:		*stats = (double)TotalPerimeter();		return S_OK;
		case CWFGM_FIRE_STAT_ROS:					*stats = MaximumROS();					return S_OK;
		case CWFGM_FIRE_STAT_CFB:					*stats = MaximumCFB();					return S_OK;
		case CWFGM_FIRE_STAT_CFC:					*stats = MaximumCFC();					return S_OK;
		case CWFGM_FIRE_STAT_SFC:					*stats = MaximumSFC();					return S_OK;
		case CWFGM_FIRE_STAT_TFC:					*stats = MaximumTFC();					return S_OK;
		case CWFGM_FIRE_STAT_FI:					*stats = MaximumFI();					return S_OK;
		case CWFGM_FIRE_STAT_HFI:					*stats = MaximumHFI();					return S_OK;
		case CWFGM_FIRE_STAT_HCFB:					*stats = MaximumHCFB();					return S_OK;
		case CWFGM_FIRE_STAT_FLAMELENGTH:			*stats = MaximumFlameLength();			return S_OK;
		case CWFGM_FIRE_STAT_MAXIMUM_BURN_DISTANCE:	*stats = MaximumBurnDistance();			return S_OK;
	}
	*stats = 0.0;
	weak_assert(0);
	return ERROR_FIRE_STAT_UNKNOWN;
}


template<class _type>
HRESULT FireFrontStats<_type>::RetrieveStat(const std::uint16_t stat, const double greater_equal, const double less_than, double*stats) const {
	HRESULT hr;
	std::uint32_t cnt = 0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if (!fp->m_status) {
			double s;
			if (FAILED(hr = fp->RetrieveStat(stat, s)))
				return hr;
			if ((s >= greater_equal) && (s < less_than))
				cnt++;
		}
		fp = fp->LN_Succ();
	}
	*stats = (double)cnt;
	return S_OK;
}

#include "InstantiateClasses.cpp"
