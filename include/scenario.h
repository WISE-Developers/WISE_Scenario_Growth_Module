/**
 * WISE_Scenario_Growth_Module: scenario.h
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

#ifndef __SCENARIO_H
#define __SCENARIO_H

#include "firestatecache.h"
#include "ScenarioExportRules.h"
#include "ScenarioAsset.h"
#include <vector>


template<class _type>
class ScenarioFire;

template<class _type>
class ActiveFire : public RefNode<ScenarioFire<_type>> {
public:
	ActiveFire();
	ActiveFire(ActiveFire<_type>* master);

	ActiveFire<_type> *LN_Succ() const { return (ActiveFire<_type>*)MinNode::LN_Succ(); };
	ActiveFire<_type> *LN_Pred() const { return (ActiveFire<_type>*)MinNode::LN_Pred(); };

	ActiveFire<_type> *m_mate_next, *m_mate_prev;
	ActiveFire<_type> *m_master;
	ActiveFire<_type> *Mate() const { return m_mate_next; }
	void Attach(ActiveFire<_type> *mate);
	bool Attached(ActiveFire<_type> *mate) const;
	void Detach();

	WTime						m_startTime, m_endTime;							// these are temporaries, used to calculate the next timestep for this fire, starting at m_startTime, ending at m_endTime
	XY_RectangleTempl<_type>	m_boundingBox;									// bounding box for LN_Ptr() when LN_Ptr() is assigned (before any growth is applied)
	std::uint32_t				m_advanced : 1,
								m_calc_st;

	DECLARE_OBJECT_CACHE_MT(ActiveFire<_type>, ActiveFire)
};


template<class _type>
struct growVoxelParms;
template<class _type>
struct growPointStruct;
template<class _type>
class DelaunayTree;


template<class _type>
class Scenario : public ScenarioCache<_type> {
	using XYPointType = XY_PointTempl<_type>;
	using XYZPointType = XYZ_PointTempl<_type>;
	using DelaunayType = DelaunayTree<_type>;

	using XYRectangleType = typename ScenarioCache<_type>::XYRectangleType;
	using XYPolyLLType = XY_PolyLL_BaseTempl<_type>;
	using XYPolyNodeType = XY_PolyLLNode<_type>;
	using XYPolyRefType = XY_PolyLLDistRef<_type>;

	std::uint64_t GetProcessTickCount();

public:
	using ScenarioGridCache<_type>::m_scenario;
	using ScenarioGridCache<_type>::RecordTimeStep;
	using ScenarioCache<_type>::m_specifiedFMC_Landscape;
	using ScenarioCache<_type>::m_specifiedElev_Landscape;
	using ScenarioCache<_type>::m_coordinateConverter;
	using ScenarioCache<_type>::GetFuel;
	using ScenarioCache<_type>::GetCorrectedFuel;
	using ScenarioCache<_type>::IsNonFuel;
	using ScenarioCache<_type>::GetFuel_NotCached;
	using ScenarioGridCache<_type>::toInternal;
	using ScenarioGridCache<_type>::fromInternal;
	using ScenarioGridCache<_type>::toInternal1D;
	using ScenarioGridCache<_type>::toInternal2D;
	using ScenarioGridCache<_type>::toInternal3D;
	using ScenarioGridCache<_type>::gridToInternal1D;
	using ScenarioGridCache<_type>::fromInternal1D;
	using ScenarioGridCache<_type>::fromInternal2D;
	using ScenarioGridCache<_type>::fromInternal3D;
	using ScenarioGridCache<_type>::gridFromInternal1D;
	using ScenarioGridCache<_type>::resolution;
	using ScenarioGridCache<_type>::resolution2;
	using ScenarioGridCache<_type>::start_ll;
	using ScenarioGridCache<_type>::start_ur;

public:
	Scenario(class CCWFGM_Scenario *scenario, const XY_Point &start_ll, const XY_Point &start_ur, const _type resolution, const double landscapeFMC, const double landscapeElev);
	~Scenario();

	MinListTempl<ScenarioTimeStep<_type>>		m_timeSteps;		// list of ScenarioTimeStep's
	RefList<ScenarioFire<_type>, ActiveFire<_type>>	m_activeFires;
	CRWThreadSemaphore							m_llLock, m_stepLock;
	HRESULT										m_stepState;

	WTime CurrentTime() const;

	HRESULT Step();
	HRESULT StepBack();

	HRESULT GetNumSteps(std::uint32_t *size) const;
	HRESULT GetStepsArray(std::uint32_t *size, std::vector<WTime> *times) const;
	HRESULT GetNumFires(std::uint32_t *count, WTime *time) const;
	HRESULT GetIgnition(std::uint32_t fire, WTime *time, boost::intrusive_ptr<CCWFGM_Ignition> *ignition) const;
	HRESULT GetVectorSize(std::uint32_t fire, WTime *time, std::uint32_t *size) const;
	HRESULT GetVectorArray(std::uint32_t fire, WTime *time, std::uint32_t *size, XY_Poly &xy_pairs) const;
	HRESULT GetStatsArray(const std::uint32_t fire, WTime *time, const std::uint16_t stat, std::uint32_t *size, std::vector<double> &stats) const;
	HRESULT GetStats(const std::uint32_t fire, ICWFGM_Fuel *fuel, WTime *time, const std::uint16_t stat, const std::uint16_t discretization, PolymorphicAttribute *stats) const;
	HRESULT GetStats(const std::uint32_t fire, WTime *time, const std::uint16_t stat, const bool only_displayable, const double greater_equal, const double less_than, double *stats) const;

	HRESULT GetBurningBox(WTime *time, XYRectangleType &bbox) const;
	HRESULT PointBurned(const XYPointType &pt, WTime *time, bool *status) const;

	HRESULT GetStats(const XY_Point &min_utmpt, const XY_Point& max_utmpt, const XYPointType& pt1, const XYPointType& pt2, WTime* mintime, WTime* time, std::uint16_t stat_cnt, std::uint16_t* stats_array, NumericVariant* vstats, bool only_displayable, std::uint32_t technique, std::uint16_t discretize, bool test);

	HRESULT Export(const CCWFGM_Ignition *set, WTime *start_time, WTime *end_time, std::uint16_t flags, const TCHAR *driver_name, const TCHAR *projection, const TCHAR *file_path, const ScenarioExportRules &rules, ScenarioTimeStep<_type> *_sts = nullptr) const;
	HRESULT ExportCriticalPath(const AssetNode<_type>* node, const AssetGeometryNode<_type>* g, const std::uint16_t flags, const TCHAR* driver_name, const TCHAR* csProjection, const TCHAR* file_path, const ScenarioExportRules& rules) const;
	HRESULT BuildCriticalPath(const AssetNode<_type>* node, const AssetGeometryNode<_type>* g, const std::uint16_t flags, CriticalPath* polyset, const ScenarioExportRules* rules) const;

	std::vector<class FirePoint<_type>*>	m_omp_fp_array;
	std::vector<class FireFront<_type>*>	m_omp_ff_array;
	growVoxelParms<_type>					*m_omp_gvs_array;
	growPointStruct<_type>					*m_omp_gps_array;
	std::uint32_t							m_omp_gvs_array_size,
											m_omp_gps_array_size;

private:
	HRESULT GetStep(WTime *time, ScenarioTimeStep<_type> **sts, const bool only_displayable) const;
	ScenarioTimeStep<_type>* GetPreviousStep(ScenarioTimeStep<_type>* sts, bool only_displayable, const FireFront<_type> *ff) const;
	ScenarioTimeStep<_type>* GetPreviousDisplayStep(ScenarioTimeStep<_type>* sts, FireFront<_type>* closest_ff, ScenarioTimeStep<_type>* prev_sts) const;
	ScenarioTimeStep<_type>* Purge();

	void buildDelaunay2(const WTime &mintime, const WTime &t, const XYPointType &pt, bool only_displayable, DelaunayType *dt); // this is here for testing purposes, it will hopefully outperform buildDelaunay(), and eventually replace it.

	HRESULT getCalculatedStats(XYPointType c_pt, const WTime& time, ICWFGM_Fuel*& fuel, const CCWFGM_FuelOverrides &overrides, bool valid, std::uint64_t& flags, const std::uint32_t technique,
		double& fbp_rss, double& fbp_roseq, double& fbp_ros, double& fbp_fros, double& fbp_bros, double& fbp_raz, double* fbp_rosv = nullptr, double* fbp_v = nullptr,
		double* cfb = nullptr, double* cfc = nullptr, double* rso = nullptr, double* csi = nullptr, double* sfc = nullptr, double* tfc = nullptr, double* fi = nullptr, double* flameLength = nullptr, double* fmc = nullptr);

	struct stats_key {
		XYPointType pt;
		std::uint64_t mintime;
		std::uint64_t time;
	};

public:
	struct closest_calc {
		FirePoint<_type> *fp;
		FireFront<_type> *ff;
		std::uint64_t time;
	};

private:
	ValueCacheTempl_MT<stats_key, closest_calc> m_closestcache;

protected:
	HRESULT getStatsCalculate(const XYPointType& pt, WTime* time, std::uint16_t stat_cnt, USHORT* stats_array, NumericVariant* vstats, const std::uint32_t technique, const bool only_displayable);
	HRESULT getStatsClosestVertex(const XYPointType& pt, WTime* mintime, WTime* time, std::uint16_t stat_cnt, USHORT* stats_array, NumericVariant* vstats, const bool only_displayable, bool test);
	HRESULT getStatsDiscretize(const XYPointType& min_pt, const XYPointType& max_pt, WTime* mintime, WTime* time, std::uint16_t stat_cnt, USHORT* stats_array, NumericVariant* vstats, const bool only_displayable, USHORT discretize);
	HRESULT getStatsInterpolate(const XY_Point& utmpt, const XYPointType& pt, WTime* mintime, WTime* time, std::uint16_t stat_cnt, USHORT* stats_array, NumericVariant* vstats, const bool only_displayable, const std::uint32_t technique);

public:
	double m_tinv;

	DECLARE_OBJECT_CACHE_MT(Scenario<_type>, Scenario)
};

#endif
