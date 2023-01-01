/**
 * WISE_Scenario_Growth_Module: ScenarioAsset.cpp
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

#include "FireEngine.h"
#include "ICWFGM_Fuel.h"
#include "ScenarioAsset.h"
#include "ScenarioTimeStep.h"
#include "scenario.h"
#include "CWFGM_Scenario.h"
#include "FireEngine_ext.h"
#include "poly.h"


#include "vectors.h"
#include "propsysreplacement.h"

#define EPSILON (1e-4)


template<class _type>
void AssetGeometryNode<_type>::fixClosestPoint() {
	weak_assert(m_closestFirePoint);
	if (!m_closestFirePoint)
		return;
	if (m_closestFireFront->Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE)) {
		// we may be on a timestep that will be purged
		FirePoint<_type>* fp = m_closestFirePoint;
		FireFront<_type>* ff = m_closestFireFront;
		ActiveFire<_type>* af = ff->Fire()->m_activeFire;
		const ScenarioTimeStep<_type>* sts = m_closestFireFront->Fire()->TimeStep();

		weak_assert(findFireFront(sts, af, fp) == ff);

		while (!sts->m_displayable) {
			if (!fp->m_prevPoint) {			// if this happens, then it was either the first point in a sequence, or it was a newly introduced point due to a distance
											// between 2 vertices becoming too great.
				FirePoint<_type>* pfp = fp->LN_PredWrap(), * sfp = fp->LN_SuccWrap();
				while (pfp != fp) {
					if (pfp->m_prevPoint)
						break;
					pfp = pfp->LN_PredWrap();
				}
				while (sfp != fp) {
					if (sfp->m_prevPoint)
						break;
					sfp = sfp->LN_SuccWrap();
				}
				if (pfp == sfp) {
					fp = pfp;
					if (!fp->m_prevPoint) {	// looped and couldn't find any point with a prior point, so must be the first step
						m_closestFirePoint = nullptr;
						m_closestFireFront = nullptr;
						return;
					}
				} else {
					double d1 = fp->DistanceToSquared(*pfp), d2 = fp->DistanceToSquared(*sfp);
					if (d1 < d2)
						fp = pfp;
					else
						fp = sfp;
				}
			}
			FirePoint<_type>* fp1 = fp->m_prevPoint;

			do {
				sts = sts->LN_Pred();
				ff = findFireFront(sts, af, fp1);
			} while ((!ff) && (sts));

			fp = fp1;
		}
		m_closestFirePoint = fp;
		m_closestFireFront = ff;
	}
}


template<class _type>
FireFront<_type>* AssetGeometryNode<_type>::findFireFront(const ScenarioTimeStep<_type>* sts, const ActiveFire<_type>* af, const FirePoint<_type>* point) const {
	ScenarioFire<_type>* sf = sts->m_fires.LH_Head();
	while (sf->LN_Succ()) {
		if (sf->m_activeFire == af) {
			FireFront<_type>* ff = sf->LH_Head();
			while (ff->LN_Succ()) {
				if (ff->NodeHasIndex(point))
					return ff;
				ff = ff->LN_Succ();
			}
			return nullptr;
		}
		sf = sf->LN_Succ();
	}
	return nullptr;
}


template<class _type>
void AssetGeometryNode<_type>::BuildCriticalPath(WTimeManager* manager, CriticalPath& set, const ScenarioExportRules* rules) const {
	const ScenarioTimeStep<_type>* sts = m_closestFireFront->Fire()->TimeStep();
	if (!sts)
		return;

	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(sts->m_lock), SEM_FALSE);
																						// this method will lock (for shared/read access) this sts (being the last time step for a critical path) but will
																						// not repeatedly lock all sts's going into the past to build the critical path.  So it really isn't compliant with
																						// other code that we're doing with these locks, but we're walking along past perimeters and fire states, so if we
																						// lock this one as unchanged, then the prior ones should be in an unchanged state too, for what we're doing here.
	CriticalPathPoint* polyline = set.New();
	polyline->m_publicFlags = XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_MULTIPOINT;

	CriticalPathPointData* pt = polyline->New();
	pt->x = m_closestPoint.x;
	pt->y = m_closestPoint.y;
	polyline->AddHead(pt);

	constructAttributes(rules, &m_closestPoint, m_arrivalTime, &polyline->m_attributes);
	set.AddPoly(polyline);

	FirePoint<_type>* fp = m_closestFirePoint;
	FireFront<_type>* ff = m_closestFireFront;
	ActiveFire<_type>* af = ff->Fire()->m_activeFire;

	if (sts->m_time != m_arrivalTime) {
		polyline = set.New();
		polyline->m_publicFlags = XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_MULTIPOINT;
		pt = polyline->New();
		pt->x = fp->x;
		pt->y = fp->y;
		polyline->AddHead(pt);
		constructAttributes(rules, fp, sts->m_time, &polyline->m_attributes);
		set.InsertPoly(0, polyline);
	}

	while (true) {
		FirePoint<_type>* pfp = fp->LN_PredWrap(),
						* sfp = fp->LN_SuccWrap();
		if (!fp->m_prevPoint) {			// if this happens, then it was either the first point in a sequence, or it was a newly introduced point due to a distance
											// between 2 vertices becoming too great.  This loop doesn't get used if we have a prior point we can look at.
											// The 'while' is so we can skip over neighbouring points to the left and right that have been marked as stopped.
			while (pfp != fp) {				// keep looking in the 'prev' direction for a point that has history to a prior timestep
				if (pfp->m_prevPoint)
					break;
				pfp = pfp->LN_PredWrap();
			}
			if (pfp == fp)					// if it looped through the entire polygon and found no history, then it's the first timestep, it kinda makes the logic
				return;						// after the next while loop a bit redundant, but we'll leave that there since the cost is minimal and may catch the odd
											// edge case anyway
			while (sfp != fp) {
				if (sfp->m_prevPoint)
					break;
				sfp = sfp->LN_SuccWrap();
			}
			if (pfp == sfp) {
				fp = pfp;
				if (!fp->m_prevPoint) {		// looped and couldn't find any point with a prior point, so must be the first step
					return;
				}
			}								// pfp is a valid point with history to the 'pred' and sfp is a valid point with history to the 'succ'
			else {
				if ((pfp->m_status == 0) != (sfp->m_status == 0)) {		// if only one point is active, then we pick that to continue on our history

					if (pfp->m_status == 0)						// pick the point that is moving, not the point that's stopped
						fp = pfp;
					else if (sfp->m_status == 0)
						fp = sfp;
				}

				// if we get here, then either both neighbours are moving, or both neighbours are inactive
				double d1 = fp->DistanceToSquared(*pfp), d2 = fp->DistanceToSquared(*sfp);
				if ((fabs(d1 - d2) < EPSILON)) {

					// going to try to change the logic here.  Before, we would loop looking for an active point to continue the history oh, but let's
					// try looking for history on an inactive point, if both immediate neighbours are stopped.  Just because they are stopped doesn't
					// mean there isn't history, just means no change into the future.  It changes the loop from a while to an if, so single pass.

					// if it's the first timestep, then it should drop out quickly on the comparison of pfp==sfp

					if (pfp->m_vector_ros > sfp->m_vector_ros)	// in looking at both < and >, I'm going to stick with > since it seems like although paths
						fp = pfp;								// (in a few basic examples) will diverge and converge, I'm sticking with > because it seems
					else										// some areas will be burned sooner than others, which seems more appropriate/accurate
						fp = sfp;
				}
				else if (d1 < d2)
					fp = pfp;
				else
					fp = sfp;
			}
		}

		FirePoint<_type>* fp1 = fp->m_prevPoint;

		do {
			sts = sts->LN_Pred();								// ***** not sure if we can do LN_CalcPred() here instead of LN_Pred() - won't matter for the
			ff = findFireFront(sts, af, fp1);					// current runs, but may be a simple optimization for multi-fire simulations
		} while ((!ff) && (sts));

		if ((ff) && (sts)) {
			polyline = set.New();
			polyline->m_publicFlags = XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_MULTIPOINT;
			pt = polyline->New();
			pt->x = fp1->x;
			pt->y = fp1->y;
			polyline->AddHead(pt);
			constructAttributes(rules, fp, sts->m_time, &polyline->m_attributes);
			set.InsertPoly(0, polyline);
		}

		fp = fp1;
	}
}


template<class _type>
void AssetGeometryNode<_type>::constructAttributes(const ScenarioExportRules* rules, const FirePoint<_type>* point, const WTime& time, std::vector<GDAL_Attribute>* attributes) const {
	if (!rules)
		return;

	std::uint32_t counter = 0;
	ExportRule* r = rules->GetNextAttributeName(counter);

	while (r) {
		GDAL_Attribute a;
		a.attributeName = r->name;
		if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY) {
			a.attributeValue = std::get<std::string>(r->value);
		}
		else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_DOUBLEPROPERTY) {
			double d;
			VariantToDouble_(r->value, &d);
			a.attributeValue = d;
		}
		else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_INT32PROPERTY) {
			std::int32_t i;
			VariantToInt32_(r->value, &i);
			a.attributeValue = i;
		}
		else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_INT64PROPERTY) {
			std::int64_t i;
			VariantToInt64_(r->value, &i);
			a.attributeValue = i;
		}
		else if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_STATPROPERTY) {
			HRESULT hr;
			GDALVariant empty;
			ULONGLONG op;
			VariantToUInt64_(r->value, &op);
			USHORT stat = (USHORT)(op & 0xffff);
			ULONG units = (ULONG)((op >> 32) & 0xffffffff);
			switch (stat) {
			case CWFGM_FIRE_STAT_DATETIME:	a.attributeValue = time.AsGDALTime(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
				break;
			default:						hr = point->RetrieveAttribute(stat, units, a.attributeValue);
				if (FAILED(hr))
					a.attributeValue = empty;
				break;
			}
		}
		attributes->push_back(a);
		r = rules->GetNextAttributeName(counter);
	}
}


template<class _type>
AssetNode<_type>::AssetNode() {
	m_operation = 0;
}


template<class _type>
AssetNode<_type>::~AssetNode() {
	AssetGeometryNode<_type>* node;
	while (node = m_geometry.RemHead())
		delete node;
}

template class AssetNode<fireengine_float_type>;
template class AssetGeometryNode<fireengine_float_type>;
