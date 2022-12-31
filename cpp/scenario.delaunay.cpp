/**
 * WISE_Scenario_Growth_Module: scenario.delaunay.cpp
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

#if __has_include(<mathimf.h>)
#include <mathimf.h>
#else
#include <cmath>
#endif

#include "angles.h"
#include <cpl_string.h>
#include "resource.h"
#include "scenario.h"
#include "ScenarioTimeStep.h"
#include "results.h"
#include "FireEngine_ext.h"
#include "GridCom_ext.h"
#include "FuelCom_ext.h"
#include "convert.h"
#include "macros.h"
#include "delaunay_tree_boost.hpp"
#include "delaunay_tree_boost.cpp"

#include <boost/cstdint.hpp>
#include <boost/multi_array.hpp>


template<class _type>
struct statstat {
	statstat() { stat = 0; raz_accumulator.x = raz_accumulator.y = 0; };

	std::uint16_t stat;
	union {
		_type stat_accumulator;
		XY_PointTempl<_type> raz_accumulator;
	};
};


template<class _type>
struct interpolate {
	using DelaunayType = DT_Point<_type>;

	interpolate(std::uint16_t i) {
		pt = nullptr;
		ss_cnt = i;
		ss = new statstat<_type>[i];
		time_accumulator = arrival_accumulator = weight_accumulator = 0.0;
	};
	~interpolate() { delete[] ss; }
	const DelaunayType *pt;
	std::uint16_t ss_cnt;
	struct statstat<_type>* ss;
	_type time_accumulator;
	_type arrival_accumulator;
	_type weight_accumulator;
};
	

template<class _type>
struct interpolate_area {
	interpolate_area(std::uint16_t i) {
		ss_cnt = i;
		ss = new statstat<_type>[i];
		time_accumulator = arrival_accumulator = weight_accumulator = 0.0;
	};
	~interpolate_area() { delete[] ss; }
	XY_PolySetTempl<_type>	m_cell;
	XY_PolyTempl<_type>	m_cellPoly;
	std::uint16_t ss_cnt;
	statstat<_type>* ss;
	_type time_accumulator;
	_type arrival_accumulator;
	_type weight_accumulator;
};


template<class _type>
static void __cdecl extend_forever(APTR /*aParm*/, const XY_PointTempl<_type> &p1, const XY_PointTempl<_type> &p2, XY_PointTempl<_type> *p) {
	XY_VectorTempl<_type> v(p2);
	v.Normalize();
	p->x = p1.x + 1000000.0 * v.x;	// the constant is in grid units, and needs to be addressed correctly in V7
	p->y = p1.y + 1000000.0 * v.y;
}


template<class _type>
static bool __cdecl linear_interpolate(APTR aParm, const typename interpolate<_type>::DelaunayType *p1, const typename interpolate<_type>::DelaunayType *p2) {
	struct interpolate<_type> *int_p = (struct interpolate<_type>*)aParm;
	FirePoint<_type> *neighbour;
	FireFront<_type> *neigh_fs;

	if (int_p->pt == p1) {
		neighbour = (FirePoint<_type>*)p2->m_user1;
		neigh_fs = (FireFront<_type>*)p2->m_user2;
	} else if (int_p->pt == p2) {
		neighbour = (FirePoint<_type>*)p1->m_user1;
		neigh_fs = (FireFront<_type>*)p1->m_user2;
	} else
		return true;

	if (!neighbour)			// one of our outside points
		return true;
	weak_assert(neigh_fs);

	if (neighbour->m_status)
		return true;

	_type weight = 1.0 / p1->m_location.DistanceToSquared(p2->m_location);

	for (std::uint16_t i = 0; i < int_p->ss_cnt; i++) {
		if (int_p->ss[i].stat == CWFGM_FIRE_STAT_RAZ) {
			XY_PointTempl<_type> raz(0.0, 1.0);
			raz.RotateXY_Compass(neighbour->m_fbp_raz);			// raz is stored as compass (not cartesian) radians
			raz.ScaleXY(weight);
			int_p->ss[i].raz_accumulator += raz;
		} else {
			_type stat;
			if ((int_p->ss[i].stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (int_p->ss[i].stat == CWFGM_FIRE_STAT_BURNED))
				stat = 1.0;
			else if ((int_p->ss[i].stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (int_p->ss[i].stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
				double tfc;
				HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
				weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
				weak_assert(!__isnan(tfc));
#else
				weak_assert(!_isnan(tfc));
#endif
				stat = tfc;
			}
			else if (int_p->ss[i].stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
				double sfc;
				HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
				weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
				weak_assert(!__isnan(sfc));
#else
				weak_assert(!_isnan(sfc));
#endif
				stat = sfc;
			}
			else if (int_p->ss[i].stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
				double cfc;
				HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
				weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
				weak_assert(!__isnan(cfc));
#else
				weak_assert(!_isnan(cfc));
#endif
				stat = cfc;

			}
			else {
				neighbour->RetrieveStat(int_p->ss[i].stat, stat);
			}
			int_p->ss[i].stat_accumulator += stat * weight;
		}
	}

    #ifdef _DEBUG
	std::string timestr(neigh_fs->Fire()->TimeStep()->m_time.ToString(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST));
	_type frac = neigh_fs->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
    #endif

	int_p->arrival_accumulator += neigh_fs->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST) * weight;
	int_p->time_accumulator += neigh_fs->Fire()->TimeStep()->m_time.GetTime(0) * weight;
	int_p->weight_accumulator += weight;

	return true;
}


template<class _type>
class area_node : public RefNode<const typename interpolate<_type>::DelaunayType> {
    public:
	XY_PolySetTempl<_type> region;
	XY_PolySetTempl<_type> region_clipped;

	DECLARE_OBJECT_CACHE_MT(area_node<_type>, area_node)
};


IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(area_node, area_node, 1024 / sizeof(area_node<fireengine_float_type>), false, 16, fireengine_float_type)


template<class _type>
struct accumulate_area {
	const typename interpolate<_type>::DelaunayType *pt;
	XY_PolySetTempl<_type> region;
	RefList<const typename interpolate<_type>::DelaunayType> neighbours;
};


template<class _type>
static bool __cdecl find_neighbours(APTR aParm, const typename interpolate<_type>::DelaunayType *p1, const typename interpolate<_type>::DelaunayType *p2) {
	struct accumulate_area<_type> *vor_region = (struct accumulate_area<_type>*)aParm;
	area_node<_type> *rn;

	if (vor_region->pt == p1) {
		FirePoint<_type> *neighbour = (FirePoint<_type>*)p2->m_user1;
		if (!neighbour)
			return true;
		if ((!neighbour->m_status) || (neighbour->m_fbp_ros)) {
			rn = new area_node<_type>;
			rn->LN_Ptr(p2);
		} else	rn = NULL;
	} else if (vor_region->pt == p2) {
		FirePoint<_type> *neighbour = (FirePoint<_type>*)p1->m_user1;
		if (!neighbour)
			return true;
		if ((!neighbour->m_status) || (neighbour->m_fbp_ros)) {
			rn = new area_node<_type>;
			rn->LN_Ptr(p1);
		} else	rn = NULL;
	} else	rn = NULL;

	if (rn)
		vor_region->neighbours.AddTail(rn);
	return true;
}


template<class _type>
static bool __cdecl build_area1(APTR aParm, const typename interpolate<_type>::DelaunayType *point,
	const XY_PointTempl<_type> &p1, const XY_PointTempl<_type> &p2,
	const XY_PointTempl<_type> &p3)
{
	struct accumulate_area<_type> *vor_region = (struct accumulate_area<_type>*)aParm;
	if (vor_region->pt == point) {
		XY_PolyTempl<_type> tri(3);
		tri.AddPoint(p1);
		tri.AddPoint(p2);
		tri.AddPoint(p3);
		vor_region->region.AddPoly(tri, 0);
	}
	return true;
}


template<class _type>
static bool __cdecl build_area2(APTR aParm, const typename interpolate<_type>::DelaunayType *point, const XY_PointTempl<_type> &p1, const XY_PointTempl<_type> &p2, const XY_PointTempl<_type> &p3) {
	struct accumulate_area<_type> *vor_region = (struct accumulate_area<_type>*)aParm;
	area_node<_type> *rn;
	if ((rn = (area_node<_type>*)vor_region->neighbours.FindPtr(point))) {
		XY_PolyTempl<_type> tri(3);
		tri.AddPoint(p1);
		tri.AddPoint(p2);
		tri.AddPoint(p3);
		rn->region.AddPoly(tri, 0);
	}
	return true;
}


template<class _type>
static bool __cdecl build_area3(APTR aParm, const typename interpolate<_type>::DelaunayType *point, const XY_PointTempl<_type> &p1, const XY_PointTempl<_type> &p2, const XY_PointTempl<_type> &p3) {
	FirePoint<_type> *neighbour = (FirePoint<_type>*)point->m_user1;
	FireFront<_type> *neigh_fs = (FireFront<_type>*)point->m_user2;
	if (!neighbour)
		return true;
	if (neighbour->m_status)
		return true;

	struct interpolate_area<_type> *int_p = (struct interpolate_area<_type>*)aParm;
	XY_PolyTempl<_type> tri(3);
	tri.AddPoint(p1);
	tri.AddPoint(p2);
	tri.AddPoint(p3);

	if (int_p->m_cellPoly.FastCollisionTest(tri, 0.0)) {
		XY_PolySetTempl<_type> tri_ps, diff;
		tri_ps.AddPoly(tri, false);
		diff.Clip(int_p->m_cell, tri_ps, PolysetOperation::INTERSECTION);

		std::uint32_t i;
		_type weight = 0.0;
		for (i = 0; i < diff.NumPolys(); i++) {
			_type area = diff.GetPoly(i).Area();
			if (diff.IsHole(i))
				area = 0.0 - area;
			weight += area;
		}

		if (weight > 0.0) {
			for (std::uint16_t i = 0; i < int_p->ss_cnt; i++) {

				if (int_p->ss[i].stat == CWFGM_FIRE_STAT_RAZ) {
					XY_PointTempl<_type> raz(0.0, 1.0);
					raz.RotateXY_Compass(neighbour->m_fbp_raz);
					raz.ScaleXY(weight);
					int_p->ss[i].raz_accumulator += raz;
				} else {
					_type stat;
					if ((int_p->ss[i].stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (int_p->ss[i].stat == CWFGM_FIRE_STAT_BURNED))
						stat = 1.0;
					else if ((int_p->ss[i].stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (int_p->ss[i].stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
						double tfc;
						HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(tfc));
#else
						weak_assert(!_isnan(tfc));
#endif
						stat = tfc;
					}
					else if (int_p->ss[i].stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
						double sfc;
						HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(sfc));
#else
						weak_assert(!_isnan(sfc));
#endif
						stat = sfc;
					}
					else if (int_p->ss[i].stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
						double cfc;
						HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(cfc));
#else
						weak_assert(!_isnan(cfc));
#endif
						stat = cfc;

					}
					else {
						neighbour->RetrieveStat(int_p->ss[i].stat, stat);
					}
					int_p->ss[i].stat_accumulator += stat * weight;
				}
			}

			int_p->arrival_accumulator += neigh_fs->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST) * weight;
			int_p->time_accumulator += neigh_fs->Fire()->TimeStep()->m_time.GetTime(0) * weight;
			int_p->weight_accumulator += weight;
		}
	}
	return true;
}


template<class _type>
HRESULT Scenario<_type>::getStatsCalculate(const XYPointType& pt, WTime* time, std::uint16_t stat_cnt, std::uint16_t* stats_array, NumericVariant* vstats, const std::uint32_t technique, const bool only_displayable) {
	ICWFGM_Fuel* fuel = NULL;		// this is set by getCalculatedStats
	boost::intrusive_ptr<ICWFGM_Fuel> copy_f;	// this is set by getCalculatedStats

	CCWFGM_FuelOverrides overrides;
	std::uint64_t flags = 0;			// this is set by getCalculatedStats
	double fbp_rss, fbp_roseq, fbp_ros, fbp_fros, fbp_bros, fbp_raz, fbp_rosv, fbp_v;
	double cfb, cfc, rso, csi, sfc, tfc, fi, fmc, flameLength;
	bool set;

	bool needs_stats = false;
	for (std::uint16_t i = 0; i < stat_cnt; i++) {
		if ((stats_array[i] < FUELCOM_ATTRIBUTE_SPREAD_START) || (stats_array[i] > FUELCOM_ATTRIBUTE_SPREAD_END)) {
			needs_stats = true;
			break;
		}
	}

	bool valid;
	fuel = GetFuel_NotCached(*time, pt, valid);
	if (fuel)
		GetCorrectedFuel(pt, *time, fuel, overrides);

	if (needs_stats)
		getCalculatedStats(pt, *time, fuel, overrides, valid, flags, technique, fbp_rss, fbp_roseq, fbp_ros, fbp_fros, fbp_bros, fbp_raz, &fbp_rosv, &fbp_v, &cfb, &cfc, &rso, &csi, &sfc, &tfc, &fi, &flameLength, &fmc);

	for (std::uint16_t i = 0; i < stat_cnt; i++) {
		switch (stats_array[i]) {
		case CWFGM_FIRE_STAT_FBP_RSI:		vstats[i] = fbp_rss;		break;
		case CWFGM_FIRE_STAT_FBP_ROSEQ:		vstats[i] = fbp_roseq;	break;
		case CWFGM_FIRE_STAT_FBP_ROS:		vstats[i] = fbp_ros;		break;
		case CWFGM_FIRE_STAT_FBP_FROS:		vstats[i] = fbp_fros;		break;
		case CWFGM_FIRE_STAT_FBP_BROS:		vstats[i] = fbp_bros;		break;
		case CWFGM_FIRE_STAT_RAZ:

											vstats[i] = fbp_raz;		break;
		case CWFGM_FIRE_STAT_CFB:			vstats[i] = cfb;			break;
		case CWFGM_FIRE_STAT_CFC:			vstats[i] = cfc;			break;
		case CWFGM_FIRE_STAT_SFC:			vstats[i] = sfc;			break;
		case CWFGM_FIRE_STAT_TFC:			vstats[i] = tfc;			break;
		case CWFGM_FIRE_STAT_FI:			vstats[i] = fi;				break;
		case CWFGM_FIRE_STAT_HFI:			vstats[i] = fi;				break;
		case CWFGM_FIRE_STAT_HCFB:			vstats[i] = cfb;			break;
		case CWFGM_FIRE_STAT_FLAMELENGTH:	vstats[i] = flameLength;	break;
		case CWFGM_FIRE_STAT_FMC:			vstats[i] = fmc;			break;
		case CWFGM_FIRE_STAT_ROSVECTOR:			vstats[i] = fbp_rosv;	break;
		case CWFGM_FIRE_STAT_DIRECTIONVECTOR:	vstats[i] = fbp_v;		break;
		case FUELCOM_ATTRIBUTE_CURINGDEGREE:
		case FUELCOM_ATTRIBUTE_PC:
		case FUELCOM_ATTRIBUTE_PDF:
		case FUELCOM_ATTRIBUTE_CBH:
		case FUELCOM_ATTRIBUTE_TREE_HEIGHT:
		case FUELCOM_ATTRIBUTE_FUELLOAD:
		case FUELCOM_ATTRIBUTE_CFL:			set = false;
											if (fuel) {
												PolymorphicAttribute pa;
												HRESULT hr = fuel->GetAttribute(stats_array[i], &pa);
												if (SUCCEEDED(hr)) {
													double vd;
													try {
														vd = std::get<double>(pa);
														vstats[i] = vd;
														set = true;
													}
													catch (std::bad_variant_access&) {
													}
												}
											}
											if (!set) {
												NumericVariant pa0;
												vstats[i] = pa0;
											}
											break;

		case CWFGM_SCENARIO_OPTION_GREENUP:
		case CWFGM_SCENARIO_OPTION_GRASSPHENOLOGY:
											vstats[i] = (flags & (1ull << stats_array[i])) ? true : false;
											break;
		default:							return E_INVALIDARG;
		}
	}
	return S_OK;
}


template<class _type>
HRESULT Scenario<_type>::getStatsClosestVertex(const XYPointType& pt, WTime *mintime, WTime* time, std::uint16_t stat_cnt, USHORT* stats_array, NumericVariant* vstats, const bool only_displayable, bool test) {
	_type stats;
	if (!test) {
		_type junk;
		_type frac1 = fabs(modf(pt.x, &junk) - 0.5);
		_type frac2 = fabs(modf(pt.y, &junk) - 0.5);
		if ((frac1 < 0.00001) && (frac2 < 0.00001) && (ScenarioGridCache<_type>::m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_CACHE_GRID_POINTS))) {
			CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore*)&m_llLock, SEM_FALSE);
			FirePoint<_type>* fp;
			FireFront<_type>* ff;
			bool good = RetrieveCPoint(pt, *mintime, *time, only_displayable, &fp, &ff);

			if (good) {	// ff and fp are valid pointers
				bool valid;
				ICWFGM_Fuel* fuel = GetFuel_NotCached(*time, pt, valid);
				bool result = false;
				HRESULT hr;
				if ((!valid) || (!fuel) || (FAILED(hr = fuel->IsNonFuel(&result))) || (result)) {
					for (std::uint16_t i = 0; i < stat_cnt; i++) {
						double zero = 0.0;
						vstats[i] = zero;
					}
					return S_OK;
				}

				for (std::uint16_t i = 0; i < stat_cnt; i++) {
					USHORT stat = stats_array[i];
					if (stat != CWFGM_FIRE_STAT_TIME) {
						if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED))
							stats = 1.0;
						else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
							double tfc;
							HRESULT hr = fp->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
							weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
							weak_assert(!__isnan(tfc));
#else
							weak_assert(!_isnan(tfc));
#endif
							stats = tfc;
						}
						else if (stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
							double sfc;
							HRESULT hr = fp->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
							weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
							weak_assert(!__isnan(sfc));
#else
							weak_assert(!_isnan(sfc));
#endif
							stats = sfc;
						}
						else if (stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
							double cfc;
							HRESULT hr = fp->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
							weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
							weak_assert(!__isnan(cfc));
#else
							weak_assert(!_isnan(cfc));
#endif
							stats = cfc;

						}
						else
							fp->RetrieveStat(stat, stats);

						if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
							(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
							(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
							(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
							stats *= resolution2();	// convert it to kg, and for # meters that this sub-cell represents

							if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
								const double HeatOfCombustion = 18000.0; // kJ/kg
								stats *= HeatOfCombustion;
								WTime mtime(*mintime);
								if (mtime < m_scenario->m_startTime)
									mtime = m_scenario->m_startTime;
								WTimeSpan delta = (*time) - mtime;
								stats /= delta.GetSecondsFraction();
							}
						}
					} else {
						stats = ff->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
					}
					vstats[i] = (double)stats;
				}

				*time = ff->Fire()->TimeStep()->m_time;
				return S_OK;
			}

			for (std::uint16_t i = 0; i < stat_cnt; i++)
				vstats[i] = 0.0;
			return ERROR_POINT_NOT_IN_FIRE;
		}
	}

	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore*)&m_llLock, SEM_FALSE);

	stats_key key;
	key.pt = pt;
	key.mintime = mintime->GetTotalSeconds();
	key.time = time->GetTotalSeconds();

	if (!test) {
		closest_calc cc, * result;
		if (result = m_closestcache.Retrieve(&key, &cc)) {
			for (std::uint16_t i = 0; i < stat_cnt; i++) {
				std::uint16_t stat = stats_array[i];
				if (stat != CWFGM_FIRE_STAT_TIME) {
					if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED))
						stats = 1.0;
					else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
						double tfc;
						HRESULT hr = result->fp->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(tfc));
#else
						weak_assert(!_isnan(tfc));
#endif
						stats = tfc;
					}
					else if (stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
						double sfc;
						HRESULT hr = result->fp->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(sfc));
#else
						weak_assert(!_isnan(sfc));
#endif
						stats = sfc;
					}
					else if (stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
						double cfc;
						HRESULT hr = result->fp->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
						weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
						weak_assert(!__isnan(cfc));
#else
						weak_assert(!_isnan(cfc));
#endif
						stats = cfc;

					}
					else
						result->fp->RetrieveStat(stat, stats);

					if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
						stats *= resolution2();	// convert it to kg, and for # meters that this sub-cell represents

						if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
							const double HeatOfCombustion = 18000.0; // kJ/kg
							stats *= HeatOfCombustion;
							WTime mtime(*mintime);
							if (mtime < m_scenario->m_startTime)
								mtime = m_scenario->m_startTime;
							WTimeSpan delta = (*time) - mtime;
							stats /= delta.GetSecondsFraction();
						}
					}
				}  else {
					stats = result->ff->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
				}
				vstats[i] = (double)stats;
			}
			WTime nt(result->time, time->GetTimeManager());
			*time = nt;
			return S_OK;
		}
	}

	ScenarioTimeStep<_type>* sts = m_timeSteps.LH_Head(), * sts_prev;
	while (sts->LN_Succ()) {
		if (((!time->GetTime(0)) || (sts->m_time <= (*time))) && ((!mintime->GetTime(0)) || (sts->m_time > (*mintime)))) {
			LONG l;
			CRWThreadSemaphoreEngage _semaphore_engageT(sts->m_lock, FALSE);
			if ((!only_displayable) || (sts->m_displayable)) {

				if (l = sts->PointInArea(pt)) {
				RESTART:
					FireFront<_type>* closest_ff;
					FirePoint<_type>* closest = sts->GetNearestPoint(pt, true, &closest_ff, true);
					sts_prev = GetPreviousStep(sts, only_displayable, closest_ff);

					if (!closest) {
						sts = sts_prev;
						if (sts_prev)
							goto RESTART;
						for (std::uint16_t i = 0; i < stat_cnt; i++)
							vstats[i] = 0.0;
						return ERROR_POINT_NOT_IN_FIRE;
					}

					bool valid;
					ICWFGM_Fuel* fuel = GetFuel_NotCached(*time, pt, valid);
					bool result = false;
					HRESULT hr;
					if ((!valid) || (!fuel) || (FAILED(hr = fuel->IsNonFuel(&result))) || (result)) {
						double zero = 0.0;
						for (std::uint16_t i = 0; i < stat_cnt; i++)
							vstats[i] = zero;
						return S_OK;
					}

					if ((sts_prev) && (sts_prev->m_time <= (*mintime)))
						sts_prev = nullptr;

					FirePoint<_type>* prev_closest;
					FireFront<_type>* prev_closest_ff;
					if (sts_prev)	prev_closest = sts_prev->GetNearestPoint(pt, false, &prev_closest_ff, false);
					else			prev_closest = nullptr;

					FirePoint<_type>* fp;
					FireFront<_type>* ff;
					if ((prev_closest) && (prev_closest->DistanceToSquared(pt) < closest->DistanceToSquared(pt))) {
						fp = prev_closest;
						ff = prev_closest_ff;
						*time = sts_prev->m_time;
					} else {
						fp = closest;
						ff = closest_ff;
						*time = sts->m_time;
					}

					for (std::uint16_t i = 0; i < stat_cnt; i++) {
						USHORT stat = stats_array[i];
						if (stat != CWFGM_FIRE_STAT_TIME) {
							if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED))
								stats = 1.0;
							else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
								double tfc;
								HRESULT hr = fp->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
								weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
								weak_assert(!__isnan(tfc));
#else
								weak_assert(!_isnan(tfc));
#endif
								stats = tfc;
							}
							else if (stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
								double sfc;
								HRESULT hr = fp->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
								weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
								weak_assert(!__isnan(sfc));
#else
								weak_assert(!_isnan(sfc));
#endif
								stats = sfc;
							}
							else if (stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
								double cfc;
								HRESULT hr = fp->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
								weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
								weak_assert(!__isnan(cfc));
#else
								weak_assert(!_isnan(cfc));
#endif
								stats = cfc;

							}
							else
								fp->RetrieveStat(stat, stats);

							if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
								(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
								(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
								(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
								stats *= resolution2();	// convert it to kg, and for # meters that this sub-cell represents

								if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
									const double HeatOfCombustion = 18000.0; // kJ/kg
									stats *= HeatOfCombustion;
									WTime mtime(*mintime);
									if (mtime < m_scenario->m_startTime)
										mtime = m_scenario->m_startTime;
									WTimeSpan delta = (*time) - mtime;
									stats /= delta.GetSecondsFraction();
								}
							}
						} else {
							stats = ff->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
						}
						vstats[i] = (double)stats;
					}

					closest_calc cc;
					cc.fp = fp;
					cc.ff = ff;
					cc.time = time->GetTotalSeconds();
					m_closestcache.Store(&key, &cc);

					return S_OK;
				}
			}
		}
		sts = sts->LN_Succ();
	}
	for (std::uint16_t i = 0; i < stat_cnt; i++)
		vstats[i] = 0.0;
	return ERROR_POINT_NOT_IN_FIRE;
}


template<class _type>
HRESULT Scenario<_type>::getStatsDiscretize(const XYPointType& pt1, const XYPointType& pt2, WTime* mintime, WTime* time, std::uint16_t stat_cnt, USHORT* stats_array, NumericVariant* vstats, const bool only_displayable, USHORT discretize) {
	if (discretize == 0)
		return E_INVALIDARG;

	boost::multi_array<Scenario<_type>::closest_calc, 2> pt_array(boost::extents[discretize][discretize]);

	for (std::uint16_t x = 0; x < discretize; x++)
		for (std::uint16_t y = 0; y < discretize; y++) {
			pt_array[x][y].ff = nullptr;
			pt_array[x][y].fp = nullptr;
			pt_array[x][y].time = 0;
		}

	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore*)&m_llLock, SEM_FALSE);

	XYPointType p;
	const _type xstep = (pt2.x - pt1.x) / (_type)discretize;
	const _type ystep = (pt2.y - pt1.y) / (_type)discretize;
	p.x = pt1.x + xstep * 0.5;
	p.y = pt1.y + ystep * 0.5;
	_type py = p.y;

	bool found = false;
	for (std::uint16_t x = 0; x < discretize; x++, p.x += xstep) {
		p.y = py;
		for (std::uint16_t y = 0; y < discretize; y++, p.y += ystep) {

			bool valid;
			ICWFGM_Fuel* fuel = GetFuel_NotCached(*time, p, valid);
			bool result = false;
			HRESULT hr;
			if ((!valid) || (!fuel) || (FAILED(hr = fuel->IsNonFuel(&result))) || (result)) {
				// if it's non-fuel then don't look for any vertex to pull data from
			}
			else {

				bool point_found = false;
				ScenarioTimeStep<_type>* sts = m_timeSteps.LH_Head(), * sts_prev;

				while (sts->LN_Succ() && (!point_found)) {
					{
						LONG l;
						CRWThreadSemaphoreEngage _semaphore_engageT(sts->m_lock, SEM_FALSE);
						if ((!only_displayable) || (sts->m_displayable)) {

							if (l = sts->PointInArea(p)) {
								point_found = true;
								FireFront<_type>* closest_ff;
								FirePoint<_type>* closest = sts->GetNearestPoint(p, true, &closest_ff, true);

RESTART:
								sts_prev = GetPreviousStep(sts, only_displayable, closest_ff);

								if (!closest) {
									sts = sts_prev;
									if (sts_prev)
										goto RESTART;
									for (std::uint16_t i = 0; i < stat_cnt; i++)
										vstats[i] = 0.0;
									weak_assert(0);
									return ERROR_POINT_NOT_IN_FIRE;
								}

								LONG inside_break;
									if (sts_prev)
										inside_break = sts_prev->pointInsideVectors(p);
									else
										inside_break = sts->pointInsideVectors(p);

								if (!inside_break) {
									FirePoint<_type>* prev_closest;
									FireFront<_type>* prev_closest_ff;
									if (sts_prev)	prev_closest = sts_prev->GetNearestPoint(p, false, &prev_closest_ff, false);
									else			prev_closest = nullptr;

									if ((prev_closest)) {
										pt_array[x][y].fp = prev_closest;
										pt_array[x][y].ff = prev_closest_ff;
										pt_array[x][y].time = sts_prev->m_time.GetTime(0);
									}
									else {
										pt_array[x][y].fp = closest;
										pt_array[x][y].ff = closest_ff;
										pt_array[x][y].time = sts->m_time.GetTime(0);
									}
									found = true;
								}
							}
						}
					}
					sts = sts->LN_Succ();
				}
			}
		}
	}

	if (found) {
		for (std::uint16_t i = 0; i < stat_cnt; i++) {
			USHORT stat = stats_array[i];
			ULONG cnt = 0;
			_type stats = 0.0, stats_1 = 0.0, min_stats = -1.0, max_stats = -1.0;
			for (std::uint16_t x = 0; x < discretize; x++)
				for (std::uint16_t y = 0; y < discretize; y++) {
					if (pt_array[x][y].fp) {
						const ScenarioTimeStep<_type>* sts = pt_array[x][y].ff->Fire()->TimeStep();
						if (sts->m_time <= (*mintime))
							continue;
						if (sts->m_time > (*time))
							continue;
						cnt++;
						if (stat != CWFGM_FIRE_STAT_TIME) {
							if (stat != CWFGM_FIRE_STAT_RAZ) {
								if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED))
									stats += 1.0;
								else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
									double tfc;
									HRESULT hr = pt_array[x][y].fp->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
									weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
									weak_assert(!__isnan(tfc));
#else
									weak_assert(!_isnan(tfc));
#endif
									stats += tfc;
								}
								else if (stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
									double sfc;
									HRESULT hr = pt_array[x][y].fp->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
									weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
									weak_assert(!__isnan(sfc));
#else
									weak_assert(!_isnan(sfc));
#endif
									stats += sfc;
								}
								else if (stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
									double cfc;
									HRESULT hr = pt_array[x][y].fp->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
									weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
									weak_assert(!__isnan(cfc));
#else
									weak_assert(!_isnan(cfc));
#endif
									stats += cfc;

								}
								else {
									double _stats;
									pt_array[x][y].fp->RetrieveStat(stat, _stats);
									stats += _stats;
								}
							}
							else {
								XYPointType raz(0.0, 1.0);
								raz.RotateXY_Compass(stats);			// raz is stored as compass (not cartesian) radians
								stats += raz.x;
								stats += raz.y;
							}
						}
						else {
							_type ttt = pt_array[x][y].ff->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
							stats += ttt;
							if ((min_stats == -1.0) || (min_stats > ttt))
								min_stats = ttt;
							if ((max_stats == -1.0) || (max_stats < ttt))
								max_stats = ttt;
						}
					}
				}


			if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED)) {
				weak_assert(stats <= (discretize * discretize));
				stats /= (discretize * discretize);
				vstats[i] = (double)stats;
			}
			else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
				(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
				(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
				(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
				stats /= (discretize * discretize);
				stats *= resolution2();

				if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
					const _type HeatOfCombustion = 18000.0; // kJ/kg
					stats *= HeatOfCombustion;
					WTime mtime(*mintime);
					if (mtime < ScenarioGridCache<_type>::m_scenario->m_startTime)
						mtime = ScenarioGridCache<_type>::m_scenario->m_startTime;
					WTimeSpan delta = (*time) - mtime;
					stats /= delta.GetSecondsFraction();
				}
				vstats[i] = (double)stats;
			}
			else if (stat == CWFGM_FIRE_STAT_ENTRY_TIME)
				vstats[i] = (double)min_stats;
			else if (stat == CWFGM_FIRE_STAT_EXIT_TIME)
				vstats[i] = (double)max_stats;
			else if (stat != CWFGM_FIRE_STAT_RAZ)
				vstats[i] = (double)(stats / (_type)cnt);
			else
				vstats[i] = (double)CARTESIAN_TO_COMPASS_RADIAN(atan2(stats_1, stats));
		}
		return S_OK;
	}
	for (std::uint16_t i = 0; i < stat_cnt; i++)
		vstats[i] = 0.0;
	return ERROR_POINT_NOT_IN_FIRE;
}


template<class _type>
HRESULT Scenario<_type>::getStatsInterpolate(const XY_Point& utmpt, const XYPointType& pt, WTime *mintime, WTime* time, std::uint16_t stat_cnt, USHORT* stats_array, NumericVariant* vstats, const bool only_displayable, const std::uint32_t technique) {
	_type stats;
	bool inside = false;

	CRWThreadSemaphoreEngage _semaphore_engage(*(CRWThreadSemaphore*)&m_llLock, SEM_FALSE);
	USHORT cnt = 0;
	ScenarioTimeStep<_type>* sts = m_timeSteps.LH_Tail();
	while (sts->LN_Pred()) {
		if ((!time->GetTime(0)) || ((sts->m_time <= (*time)) && (sts->m_time > (*mintime)))) {
			if (cnt > 3)
				break;
			else if (!cnt) {
				bool valid;
				if (IsNonFuel(sts->m_time, pt, valid)) {
					break;
				}
			}
			CRWThreadSemaphoreEngage _semaphore_engageT(sts->m_lock, FALSE);
			if (sts->PointInArea(pt)) {				// generally speaking, fires only burn out, but sometimes they will "recede" very slightly,
				inside = true;						// thanks to an artefact of the smoothing operation
				break;
			}
			cnt++;
		}
		sts = sts->LN_Pred();
	}
	if (!inside) {
		*time = ScenarioGridCache<_type>::m_scenario->m_startTime;
		for (std::uint16_t i = 0; i < stat_cnt; i++)
			vstats[i] = 0.0;
		return ERROR_POINT_NOT_IN_FIRE;
	}

	bool valid;
	ICWFGM_Fuel* fuel = GetFuel_NotCached(*time, pt, valid);
	bool result = false;
	HRESULT hr;
	if ((!valid) || (!fuel) || (FAILED(hr = fuel->IsNonFuel(&result))) || (result)) {
		double zero = 0.0;
		for (std::uint16_t i = 0; i < stat_cnt; i++)
			vstats[i] = zero;
		return S_OK;
	}

	DelaunayType _del, * m_delaunay = &_del;

	_type res = 1.0;
	fromInternal1D(res);

	XYPointType ll(sts->current_ll()), ur(sts->current_ur());
	toInternal(ll);
	toInternal(ur);
	ULONG plotx = ur.x - ll.x;
	ULONG ploty = ur.y - ll.y;
	ULONG plot = plotx > ploty ? plotx : ploty;

	_type scale = 1000.0;
	while ((plot * res * scale) > INT32_MAX)
		scale /= 10.0;
	_del.setScale(scale);
	
	buildDelaunay2(*mintime, *time, pt, only_displayable, m_delaunay);

	if (technique == SCENARIO_XYSTAT_TECHNIQUE_IDW) {
		interpolate<_type> int_p(stat_cnt);
		for (std::uint16_t i = 0; i < stat_cnt; i++)
			int_p.ss[i].stat = stats_array[i];

		int_p.pt = m_delaunay->InsertPoint(pt, nullptr, nullptr);
		if (!int_p.pt->m_user1) {
			m_delaunay->TraceDelaunayEdges(&int_p, linear_interpolate<_type>);
			if (int_p.weight_accumulator == 0.0) {
				*time = ScenarioGridCache<_type>::m_scenario->m_startTime;
				for (USHORT i = 0; i < stat_cnt; i++)
					vstats[i] = 0.0;
				return S_OK;
			}
			for (USHORT i = 0; i < stat_cnt; i++) {
				USHORT stat = stats_array[i];
				if (stat == CWFGM_FIRE_STAT_RAZ) {
					stats = CARTESIAN_TO_COMPASS_RADIAN(atan2(int_p.ss[i].raz_accumulator.y, int_p.ss[i].raz_accumulator.x));
				} else if (stat == CWFGM_FIRE_STAT_TIME)
					stats = int_p.arrival_accumulator / int_p.weight_accumulator;
				else {
					stats = int_p.ss[i].stat_accumulator / int_p.weight_accumulator;

					if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
						stats *= resolution2();	// convert it to kg, and for # meters that this sub-cell represents

						if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
							const double HeatOfCombustion = 18000.0; // kJ/kg
							stats *= HeatOfCombustion;
							WTime mtime(*mintime);
							if (mtime < m_scenario->m_startTime)
								mtime = m_scenario->m_startTime;
							WTimeSpan delta = (*time) - mtime;
							stats /= delta.GetSecondsFraction();
						}
					}
				}
				vstats[i] = (double)stats;
			}
			*time = WTime((ULONGLONG)(int_p.time_accumulator / int_p.weight_accumulator), time->GetTimeManager());
		} else {
			FirePoint<_type>* neighbour = (FirePoint<_type>*)int_p.pt->m_user1;
			FireFront<_type>* neigh_fs = (FireFront<_type>*)int_p.pt->m_user2;

			for (USHORT i = 0; i < stat_cnt; i++) {
				USHORT stat = stats_array[i];
				if (stat == CWFGM_FIRE_STAT_TIME) {
					stats = neigh_fs->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
				} else {
					neighbour->RetrieveStat(stat, stats);

					if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
						stats *= resolution2();	// convert it to kg, and for # meters that this sub-cell represents

						if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
							const double HeatOfCombustion = 18000.0; // kJ/kg
							stats *= HeatOfCombustion;
							WTime mtime(*mintime);
							if (mtime < m_scenario->m_startTime)
								mtime = m_scenario->m_startTime;
							WTimeSpan delta = (*time) - mtime;
							stats /= delta.GetSecondsFraction();
						}
					}
				}
				vstats[i] = (double)stats;
			}
			*time = neigh_fs->Fire()->TimeStep()->m_time;
		}
	}
	else if (technique == SCENARIO_XYSTAT_TECHNIQUE_AREA_WEIGHTING) {
		interpolate_area<_type> int_ap(stat_cnt);
		XYPointType test;

		NumericVariant nv;
		grid::AttributeValue av;
		hr = m_scenario->m_gridEngine->GetAttributeData(sts->m_scenario->m_scenario->m_layerThread,
			utmpt, sts->m_time, WTimeSpan(0),
			CWFGM_FUELGRID_ATTRIBUTE_X_START, m_scenario->m_optionFlags, &nv, &av, nullptr);
		test.x = std::get<double>(nv);
		hr = m_scenario->m_gridEngine->GetAttributeData(sts->m_scenario->m_scenario->m_layerThread,
			utmpt, sts->m_time, WTimeSpan(0),
			CWFGM_FUELGRID_ATTRIBUTE_Y_START, m_scenario->m_optionFlags, &nv, &av, nullptr);
		test.y = std::get<double>(nv);
		toInternal(test);

		_type internal1 = 1.0;
		gridToInternal1D(internal1);

		int_ap.m_cellPoly.AddPoint(test);
		test.x += internal1; int_ap.m_cellPoly.AddPoint(test);
		test.y += internal1; int_ap.m_cellPoly.AddPoint(test);
		test.x -= internal1; int_ap.m_cellPoly.AddPoint(test);
		int_ap.m_cell.AddPoly(int_ap.m_cellPoly, FALSE);

		for (std::uint16_t i = 0; i < stat_cnt; i++)
			int_ap.ss[i].stat = stats_array[i];

		m_delaunay->TraceVoronoiEdges(&int_ap, build_area3, extend_forever);

		if (int_ap.weight_accumulator == 0.0) {
			*time = ScenarioGridCache<_type>::m_scenario->m_startTime;
			for (std::uint16_t i = 0; i < stat_cnt; i++)
				vstats[i] = 0.0;
			return S_OK;
		}
		for (std::uint16_t i = 0; i < stat_cnt; i++) {
			std::uint16_t stat = stats_array[i];
			if (stat == CWFGM_FIRE_STAT_RAZ) {
				stats = CARTESIAN_TO_COMPASS_RADIAN(atan2(int_ap.ss[i].raz_accumulator.y, int_ap.ss[i].raz_accumulator.x));
			} else if (stat == CWFGM_FIRE_STAT_TIME)
				stats = int_ap.arrival_accumulator / int_ap.weight_accumulator;
			else {
				stats = int_ap.ss[i].stat_accumulator / int_ap.weight_accumulator;

				if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
					(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
					(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
					(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
					stats *= resolution2();	// convert it to kg, and for # meters that this sub-cell represents

					if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
						const double HeatOfCombustion = 18000.0; // kJ/kg
						stats *= HeatOfCombustion;
						WTime mtime(*mintime);
						if (mtime < m_scenario->m_startTime)
							mtime = m_scenario->m_startTime;
						WTimeSpan delta = (*time) - mtime;
						stats /= delta.GetSecondsFraction();
					}
				}
			}
			vstats[i] = (double)stats;
		}
		*time = WTime((std::uint64_t)(int_ap.time_accumulator / int_ap.weight_accumulator), time->GetTimeManager());
	}
	else if (technique == SCENARIO_XYSTAT_TECHNIQUE_VORONOI_OVERLAP) {
		accumulate_area<_type> vor_region;
		vor_region.pt = m_delaunay->GetPoint(pt);
		if (!vor_region.pt) {
			vor_region.pt = m_delaunay->InsertPoint(pt, NULL, NULL);

			interpolate<_type> int_p(stat_cnt);
			for (USHORT i = 0; i < stat_cnt; i++)
				int_p.ss[i].stat = stats_array[i];

			m_delaunay->TraceDelaunayEdges(&vor_region, find_neighbours<_type>);
			m_delaunay->TraceVoronoiEdges(&vor_region, build_area1<_type>, extend_forever<_type>);
			m_delaunay->DeletePoint(vor_region.pt);
			m_delaunay->TraceVoronoiEdges(&vor_region, build_area2<_type>, extend_forever<_type>);

			area_node<_type>* an = (area_node<_type>*)vor_region.neighbours.LH_Head();
			while (an->LN_Succ()) {
				an->region_clipped.Clip(vor_region.region, an->region, PolysetOperation::INTERSECTION);

				if (an->region_clipped.NumPolys()) {
					FirePoint<_type>* neighbour = (FirePoint<_type>*)an->LN_Ptr()->m_user1;
					FireFront<_type>* neigh_fs = (FireFront<_type>*)an->LN_Ptr()->m_user2;

					_type weight = 0.0;
					for (ULONG iii = 0; iii < an->region_clipped.NumPolys(); iii++)
						weight += an->region_clipped.GetPoly(0).Area();

					for (std::uint16_t i = 0; i < stat_cnt; i++) {
						std::uint16_t stat = stats_array[i];
						_type stats;
						if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED))
							stats = 1.0;
						else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
							double tfc;
							HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
							weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
							weak_assert(!__isnan(tfc));
#else
							weak_assert(!_isnan(tfc));
#endif
							stats = tfc;
						}
						else if (stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
							double sfc;
							HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
							weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
							weak_assert(!__isnan(sfc));
#else
							weak_assert(!_isnan(sfc));
#endif
							stats = sfc;
						}
						else if (stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
							double cfc;
							HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
							weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
							weak_assert(!__isnan(cfc));
#else
							weak_assert(!_isnan(cfc));
#endif
							stats = cfc;

						}
						else {
							neighbour->RetrieveStat(stat, stats);
						}


						if (stat == CWFGM_FIRE_STAT_RAZ) {
							XYPointType raz(0.0, 1.0);
							raz.RotateXY_Compass(stats);
							raz.ScaleXY(weight);
							int_p.ss[i].raz_accumulator += raz;
						}
						else
							int_p.ss[i].stat_accumulator += stats * weight;
					}

					int_p.arrival_accumulator += neigh_fs->Fire()->TimeStep()->m_time.GetDayFractionOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST) * weight;//(neigh_fs->Fire()->TimeStep()->m_time - neigh_fs->Fire()->Ignition()->m_ignitionTime).GetDaysFraction() * weight;
					int_p.time_accumulator += neigh_fs->Fire()->TimeStep()->m_time.GetTime(0) * weight;
					int_p.weight_accumulator += weight;
				}
				an = (area_node<_type>*)an->LN_Succ();
			}

			while (an = (area_node<_type>*)vor_region.neighbours.RemHead())
				delete an;

			if (int_p.weight_accumulator == 0.0) {
				*time = ScenarioGridCache<_type>::m_scenario->m_startTime;
				for (std::uint16_t i = 0; i < stat_cnt; i++)
					vstats[i] = 0.0;
				return S_OK;
			}

			for (std::uint16_t i = 0; i < stat_cnt; i++) {
				std::uint16_t stat = stats_array[i];
				if (stat == CWFGM_FIRE_STAT_RAZ) {
					stats = CARTESIAN_TO_COMPASS_RADIAN(atan2(int_p.ss[i].raz_accumulator.y, int_p.ss[i].raz_accumulator.x));

				} else if (stat == CWFGM_FIRE_STAT_TIME)
					stats = int_p.arrival_accumulator / int_p.weight_accumulator;
				else {
					stats = int_p.ss[i].stat_accumulator / int_p.weight_accumulator;

					if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
						(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
						stats *= resolution2();	// convert it to kg, and for # meters that this sub-cell represents

						if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
							const double HeatOfCombustion = 18000.0; // kJ/kg
							stats *= HeatOfCombustion;
							WTime mtime(*mintime);
							if (mtime < m_scenario->m_startTime)
								mtime = m_scenario->m_startTime;
							WTimeSpan delta = (*time) - mtime;
							stats /= delta.GetSecondsFraction();
						}
					}
				}
				vstats[i] = (double)stats;
			}
			*time = WTime((ULONGLONG)(int_p.time_accumulator / int_p.weight_accumulator), time->GetTimeManager());
		} else {
			FirePoint<_type>* neighbour = (FirePoint<_type>*)vor_region.pt->m_user1;
			FireFront<_type>* neigh_fs = (FireFront<_type>*)vor_region.pt->m_user2;

			for (std::uint16_t i = 0; i < stat_cnt; i++) {
				std::uint16_t stat = stats_array[i];
				if ((stat == CWFGM_FIRE_STAT_BURNED_CHANGE) || (stat == CWFGM_FIRE_STAT_BURNED))
					stats = 1.0;
				else if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) || (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
					double tfc;
					HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_TFC, tfc); // units are in kg/m2
					weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
					weak_assert(!__isnan(tfc));
#else
					weak_assert(!_isnan(tfc));
#endif
					stats = tfc;
				}
				else if (stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) {
					double sfc;
					HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_SFC, sfc); // units are in kg/m2
					weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
					weak_assert(!__isnan(sfc));
#else
					weak_assert(!_isnan(sfc));
#endif
					stats = sfc;
				}
				else if (stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) {
					double cfc;
					HRESULT hr = neighbour->RetrieveStat(CWFGM_FIRE_STAT_CFC, cfc); // units are in kg/m2
					weak_assert(SUCCEEDED(hr));
#ifdef __GNUC__
					weak_assert(!__isnan(cfc));
#else
					weak_assert(!_isnan(cfc));
#endif
					stats = cfc;

				}
				else {
					neighbour->RetrieveStat(stat, stats);
				}

				if ((stat == CWFGM_FIRE_STAT_TOTAL_FUEL_CONSUMED) ||
					(stat == CWFGM_FIRE_STAT_SURFACE_FUEL_CONSUMED) ||
					(stat == CWFGM_FIRE_STAT_CROWN_FUEL_CONSUMED) ||
					(stat == CWFGM_FIRE_STAT_RADIATIVE_POWER)) {
					stats *= resolution2();	// convert it to kg, and for # meters that this sub-cell represents

					if (stat == CWFGM_FIRE_STAT_RADIATIVE_POWER) {
						const double HeatOfCombustion = 18000.0; // kJ/kg
						stats *= HeatOfCombustion;
						WTime mtime(*mintime);
						if (mtime < m_scenario->m_startTime)
							mtime = m_scenario->m_startTime;
						WTimeSpan delta = (*time) - mtime;
						stats /= delta.GetSecondsFraction();
					}
				}
				vstats[i] = (double)stats;
			}
			*time = neigh_fs->Fire()->TimeStep()->m_time;
		}
	}
	return S_OK;
}


template<class _type>
HRESULT Scenario<_type>::GetStats(const XY_Point& min_utmpt, const XY_Point& max_utmpt, const XYPointType& pt1, const XYPointType& pt2, WTime* mintime, WTime* time, std::uint16_t stat_cnt, std::uint16_t* stats_array, NumericVariant* vstats, bool only_displayable, std::uint32_t technique, std::uint16_t discretize, bool test) {
	if ((technique & 0x0fffffff) == SCENARIO_XYSTAT_TECHNIQUE_CALCULATE) {
		XYPointType pt(pt1.PointBetween(pt2));
		return getStatsCalculate(pt, time, stat_cnt, stats_array, vstats, technique, only_displayable);
	}
	else if ((technique & 0x0fffffff) == SCENARIO_XYSTAT_TECHNIQUE_CLOSEST_VERTEX) {
		XYPointType pt(pt1.PointBetween(pt2));
		return getStatsClosestVertex(pt, mintime, time, stat_cnt, stats_array, vstats, only_displayable, test);
	}
	else if ((technique & 0x0fffffff) == SCENARIO_XYSTAT_TECHNIQUE_DISCRETIZE) {
		return getStatsDiscretize(pt1, pt2, mintime, time, stat_cnt, stats_array, vstats, only_displayable, discretize);
	}
	else {
		XY_Point utmpt(min_utmpt.PointBetween(max_utmpt));
		XYPointType pt(pt1.PointBetween(pt2));
		return getStatsInterpolate(utmpt, pt, mintime, time, stat_cnt, stats_array, vstats, only_displayable, technique);
	}
}


template<class _type>
void Scenario<_type>::buildDelaunay2(const WTime &mintime, const WTime &t, const XYPointType &pt, bool only_displayable, DelaunayType *m_delaunay) {
	// determine the "radius" of area of interest around our point
	_type d = max(ScenarioGridCache<_type>::m_scenario->perimeterResolution(0.0), ScenarioGridCache<_type>::m_scenario->spatialThreshold(0.0)) * 2.0;
	gridToInternal1D(d);
	std::uint32_t loop_cnt = 0;
	bool b = false;
	RefList<XYPolyNodeType, XYPolyRefType> list;

	std::uint32_t vertex_cnt;

	do {
		vertex_cnt = m_delaunay->NumPoints();
		d *= 1.25;
		loop_cnt++;
		m_delaunay->DeletePoints();
		b = false;
		ScenarioTimeStep<_type> *sts = m_timeSteps.LH_Head();
		while (sts->LN_Succ()) {
			if (((!t.GetTime(0)) || (sts->m_time <= t)) && ((!mintime.GetTime(0)) || (sts->m_time > mintime))) {
				CRWThreadSemaphoreEngage _semaphore_engageT(sts->m_lock, SEM_FALSE);
				if ((!only_displayable) || (sts->m_displayable)) {
					ScenarioFire<_type> *sf = sts->m_fires.LH_Head();
					while (sf->LN_Succ()) {
						FireFront<_type> *fs = sf->LH_Head();
						while (fs->LN_Succ()) {
						//according to tests, this pointSet method gives us optimal results
							if (fs->FastCollisionTest(pt, d))
								fs->PointSet(pt, d, true, list, nullptr);

							XYPolyRefType *node;
							while (node = list.RemHead()) {

								if (!b) {
									_type d2 = d*3;
									XYPointType p1(pt.x-d2, pt.y-d2); FirePoint<_type> f1(p1);
									XYPointType p2(pt.x+d2, pt.y-d2); FirePoint<_type> f2(p2);
									XYPointType p3(pt.x-d2, pt.y+d2); FirePoint<_type> f3(p3);
									XYPointType p4(pt.x+d2, pt.y+d2); FirePoint<_type> f4(p4);

									m_delaunay->InsertPoint(f1, NULL, NULL); // top left
									m_delaunay->InsertPoint(f2, NULL, NULL); // top right
									m_delaunay->InsertPoint(f3, NULL, NULL); // bottom left
									m_delaunay->InsertPoint(f4, NULL, NULL); // bottom right
									b = true;
								}
								FirePoint<_type> *fp = (FirePoint<_type>*)node->LN_Ptr();
								if (((fp->m_prevPoint) && (!fp->Equals(*fp->m_prevPoint))) || ((!fp->m_prevPoint) && (!fp->m_status))) {
									if (!m_delaunay->GetPoint(*fp)) {
										m_delaunay->InsertPoint(*fp, fp, fs);

									}
								}
								delete node;
							}
							fs = fs->LN_Succ();
						}
						sf = sf->LN_Succ();
					}
				}
			}
			sts = sts->LN_Succ();
		}
	} while ((loop_cnt <= 20) && ((m_delaunay->NumPoints() < 20) || (m_delaunay->NumPoints() == vertex_cnt)));
}


#include "InstantiateClasses.cpp"
