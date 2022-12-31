/**
 * WISE_Scenario_Growth_Module: firefront.cpp
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
#include "macros.h"
#include <omp.h>
#include "CWFGM_Scenario.h"
#include "firefront.h"
#include "ScenarioTimeStep.h"
#include "scenario.h"


IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(FireFront, FireFront, 4 * 1024 * 1024 / sizeof(FireFront<fireengine_float_type>), false, 16, fireengine_float_type)
IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(FireFrontExport, FireFrontExport, 256 * 1024 / sizeof(FireFrontExport<fireengine_float_type>), false, 16, fireengine_float_type)


template<class _type>
FireFront<_type>::FireFront() {
	m_fire = NULL;
}


template<class _type>
FireFront<_type>::FireFront(const FireFront<_type>::XYPolyConstType &ff) : FireFrontStats<_type>(ff) {
	weak_assert(0);				// shouldn't ever be called
}


template<class _type>
FireFront<_type>::FireFront(const XYPolyLLType &ff) {
	m_fire = NULL;
	CopyPoints(ff, (std::uint32_t)-1);
}


template<class _type>
FireFront<_type>::FireFront(const ScenarioFire<_type> *fire) : FireFrontStats<_type>() {
	m_fire = fire;
}


template<class _type>
FireFront<_type>::FireFront(const ScenarioFire<_type> *fire, const XYPolyConstType &toCopy) : FireFrontStats<_type>(toCopy) {
	m_fire = fire;
}


template<class _type>
FireFront<_type>::FireFront(const ScenarioFire<_type> *fire, const FireFront<_type> &toCopy) : FireFrontStats<_type>() {

	this->m_publicFlags = toCopy.m_publicFlags;
	m_fire = fire;

	CopyPoints(toCopy, (fire) ? fire->m_newVertexStatus : 0);
}


template<class _type>
void FireFront<_type>::CopyPoints(const XYPolyLLType &toCopy, std::uint32_t status) {

	if (!status) {
		std::uint32_t numPoints = FirePoint<_type>::staticObjectClassFirePoint.NewSet(*((RefList<FirePoint<_type>>*)&m_ptList), toCopy.NumPoints());

		if (numPoints != toCopy.NumPoints())
			throw std::bad_alloc();

		FirePoint<_type> *n1 = LH_Head(),
			  *n2 = (FirePoint<_type> *)toCopy.LH_Head();
		while (n2->LN_Succ()) {
			n1 = new (n1) FirePoint(*n2);
			n2->m_succPoint = n1;
			n2 = n2->LN_Succ();
			n1 = n1->LN_Succ();
		}
	} else {
		if (((std::int32_t)status) < 0)					// only done when creating ignitions
			status = 0;
		std::uint32_t numPoints = FirePoint<_type>::staticObjectClassFirePoint.NewSet(*((RefList<FirePoint<_type>>*)&m_ptList), toCopy.NumPoints());

		if (numPoints != toCopy.NumPoints())
			throw std::bad_alloc();

		FirePoint<_type>			*n1 = LH_Head();
		XY_PolyLLNode<_type>		*n2 = toCopy.LH_Head();
		while (n2->LN_Succ()) {
			n1 = new (n1) FirePoint<_type>(*n2);
			n1->m_status = status;
			n2 = n2->LN_Succ();
			n1 = n1->LN_Succ();
		}
	}
}


template<class _type>
FirePoint<_type> *FireFront<_type>::New() const {			// callback for getting new firepoints during polygon set operations
	FirePoint<_type> *fp = new FirePoint<_type>();
	if (FireFrontStats<_type>::m_fire)
		fp->m_status = m_fire->m_newVertexStatus;
	else	fp->m_status = FP_FLAG_NORMAL;
	return fp;
}


template<class _type>
const XY_PolyLLNode<_type> *FireFront<_type>::ChooseToKeep(const XY_PolyLLNode<_type> *first, const XY_PolyLLNode<_type> *second) const {
	const FirePoint<_type> *fp1 = (const FirePoint<_type> *)first,
						   *fp2 = (const FirePoint<_type> *)second;
	if (fp1 == fp2) {						// we're trying to simplify the front with what we think are redundant points
		if (!fp1->m_status)
			return fp1;						// it's still active so keep it
		FirePoint<_type> *n = fp1->LN_PredWrap();
		if (!n->m_status)
			return fp1;

		n = fp1->LN_SuccWrap();
		if (!n->m_status)
			return fp1;						// this is an inactive point, but one of its neighbours is active so we need to keep it as a placeholder
		return NULL;						// it's inactive, and its neighbours are also inactive so we're okay with removing it
	}

	if (fp1->m_successful_breach)
		return fp1;
	if (fp2->m_successful_breach)
		return fp2;
	if (!fp1->m_status)
		return fp1;							// fp1 is active, so keep it (even if fp2 is active too, don't care)
	return fp2;
}


template<class _type>
bool FireFront<_type>::FindPoint_Participates(const XY_PolyLLNode<_type> *point, APTR parm) const {
	if (parm) {
		if (*((bool*)parm))
			return (((const FirePoint<_type>*)point)->m_status != FP_FLAG_FIRE) ? true : false;
	}
	return (((const FirePoint<_type>*)point)->m_status == FP_FLAG_NORMAL) ? true : false;
}


template<class _type>
FirePoint<_type> *FireFront<_type>::GetNearestPoint(const XYPointType &pt, bool all_points) {
	return (FirePoint<_type> *)ClosestPoint(pt, &all_points, nullptr);
}


#define QUEUE_UP	128

template<class _type>
bool FireFront<_type>::advancePoint(const _type scale, const FirePoint<_type> *prev, FirePoint<_type> *curr, const FirePoint<_type> *next,
    const XYPointType &prev_e_ros, const XYPointType &curr_e_ros, const XYPointType &next_e_ros) {

//	does curr_e_ros equal m_ellipse_ros?
	if (curr->CanMove()) {				// only smooth (and advance) active points

		bool retval;
		XYPointType delta_new;
		delta_new = curr->m_prevPoint->m_ellipse_ros;

		if ((delta_new.x != 0.0) || (delta_new.y != 0.0)) {
			delta_new *= scale;				// change from ROS to distance travelled in grid units

			XYPointType new_loc = *curr;
			new_loc += delta_new;

			SetPoint(curr, new_loc);			// *********** m_ellipse_ros seems to be also used by the grid tracking code, 
			retval = true;					// should remove these dependencies and turn this off
		} else
			retval = false;
		curr->m_ellipse_ros = delta_new;			// temporary, will be overwritten later in the step
		return retval;
	}
	return false;
}

template<class _type>
struct advancePointStr {
	FireFront<_type> *self;
	FirePoint<_type> * VOLATILE fp;
	CThreadCriticalSection lock_ll;
	_type scale;
};


template<class _type>
bool FireFront<_type>::AdvanceFire(const _type scale) {
	EnableCaching(false);

	bool advanced = false;
	if ((NumPoints() > QUEUE_UP) && (Fire()->TimeStep()->m_scenario->m_pool)) {

		std::vector<FirePoint<_type> *> &fp_array = Fire()->TimeStep()->m_scenario->m_omp_fp_array;
		std::int32_t i = 0, num_pts = NumPoints();
		if ((std::uint32_t)fp_array.size() < num_pts)
			fp_array.resize(num_pts);
		FirePoint<_type> *fp = LH_Head();
		while (fp->LN_Succ()) {
			fp_array[i++] = fp;
			fp = fp->LN_Succ();
		}

		int thread_id = (Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY))) ? -1 : -2;
		#pragma omp parallel for shared(advanced) num_threads(Fire()->TimeStep()->m_scenario->m_scenario->m_threadingNumProcessors) firstprivate(thread_id)
		for (i = 0; i < num_pts; i++) {
			if (thread_id == -1) {
				thread_id = omp_get_thread_num();
				CWorkerThread::native_handle_type thread = CWorkerThreadPool::GetCurrentThread();
				CWorkerThreadPool::SetThreadAffinityToMask(thread, thread_id);
			}
			FirePoint<_type> *next = fp_array[i];
			FirePoint<_type> *curr = next->LN_PredWrap();
			FirePoint<_type> *prev = curr->LN_PredWrap();
			if (advancePoint(scale, prev, curr, next, prev->m_prevPoint->m_ellipse_ros, curr->m_prevPoint->m_ellipse_ros, next->m_prevPoint->m_ellipse_ros))
				advanced = true;
		}

	} else {
		FirePoint<_type>	*curr = LH_Tail(),
							*prev = curr->LN_Pred(),
							*next = LH_Head();				// we deal with the neighbours for smoothing

		XYPointType	next_e_ros,
					prev_e_ros = prev->m_prevPoint->m_ellipse_ros,
					curr_e_ros = curr->m_prevPoint->m_ellipse_ros;

		while (next->LN_Succ()) {
			next_e_ros = next->m_prevPoint->m_ellipse_ros;
			if (advancePoint(scale, prev, curr, next, prev_e_ros, curr_e_ros, next_e_ros))
				advanced = true;
			prev_e_ros = curr_e_ros;
			curr_e_ros = next_e_ros;
			prev = curr;
			curr = next;
			next = next->LN_Succ();
		}
	}
	return advanced;
}

#include "InstantiateClasses.cpp"
