/**
 * WISE_Scenario_Growth_Module: FireStateTrack.cpp
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
#include "CWFGM_Scenario.h"
#include "firefront.h"
#include "FireEngine_ext.h"
#include "ScenarioTimeStep.h"
#include "scenario.h"
#include "raytrace.h"
#include <omp.h>


template<class _type>
struct stepVoxelStart {
	FireFront<_type> *self;

	FirePoint<_type> * VOLATILE fp;
	std::uint32_t mode;

	alignas(sizeof(_type)) _type grid_per_meter;

	CThreadCriticalSection lock_ll, lock_self;

	bool use_lock;
};


template<class _type>
struct FireStateCallback {
	FireFront<_type> *m_fs;
	FirePoint<_type> *m_fp;
	ICWFGM_GridEngine *m_grid;
	_type grid_per_meter;						// resolution of the grid
	_type m_orig_max;							// original length of the ray (from prev to curr) - we then know when we can abort
	_type m_max;								// how long the ray is (from prev to curr) - this may change if we encounter fuel breaks
	_type m_acc;								// accumulated distance as we proceed along the ray
	_type m_nf_acc;								// accumulated distance through *this* non-fuel block
	typename FireFront<_type>::XYPointType m_nf_entry;							// where we entered this non-fuel block
	std::uint32_t m_nf_status;
};


template<class _type>
int __cdecl stepVoxelCallback(APTR parameter, const XYZ_PointTempl<_type> *entry, const XYZ_PointTempl<_type> *exit) {
	return ((FireStateCallback<_type>*)parameter)->m_fs->stepVoxel(parameter, entry, exit);
}

template int stepVoxelCallback<fireengine_float_type>(APTR, const XYZ_PointTempl<fireengine_float_type>*, const XYZ_PointTempl<fireengine_float_type>*);


template<class _type>
int FireFront<_type>::stepVoxel(APTR parameter, const XYZPointType *entry, const XYZPointType *exit) {
	FireStateCallback<_type> *fscb = (FireStateCallback<_type>*)parameter;
	XYPointType _entry(entry->x, entry->y), _exit(exit->x, exit->y);
	this->Fire()->TimeStep()->m_scenario->fromInternal(_entry);
	this->Fire()->TimeStep()->m_scenario->fromInternal(_exit);
	_type	dx = exit->x - entry->x,
			dy = exit->y - entry->y;
	_type	utmdx = _exit.x - _entry.x,
			utmdy = _exit.y - _entry.y;
	XYPointType utmmid(_entry.x + utmdx * (_type)0.5, _entry.y + utmdy * (_type)0.5);

	_type dist = sqrt(dx * dx + dy * dy);

	if (dist == 0.0)
		return 1;

	bool nonfuel;
	bool allow_breaching;
	Scenario<_type> *s = Fire()->TimeStep()->m_scenario;
	if ((utmmid.x < Fire()->TimeStep()->current_ll().x) ||
		(utmmid.y < Fire()->TimeStep()->current_ll().y) ||
		(utmmid.x >= Fire()->TimeStep()->current_ur().x) ||
		(utmmid.y >= Fire()->TimeStep()->current_ur().y)) {
			nonfuel = true;					// if it's out of the range of the plot, then quickly stop it
		allow_breaching = false;
	} else {
								// if we haven't converted nonfuels to vector breaks, then check for non fuels
		bool valid;
		nonfuel = s->IsNonFuelUTM(Fire()->TimeStep()->LN_Pred()->m_time, utmmid, valid);
		allow_breaching = (s->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_BREACHING)) ? true : false;
	}

	if (nonfuel) {	// if we are in a non-fuel block...
		if (!allow_breaching) {
			fscb->m_nf_entry.x = entry->x;
			fscb->m_nf_entry.y = entry->y;
			fscb->m_nf_status = FP_FLAG_NOFUEL;
			return 0;				// hit a non-fuel and we aren't allowed to breach so change where we stop and abort
		}

		if (fscb->m_nf_acc == 0.0) {			// if we are just now entering the non-fuel block...
			fscb->m_nf_entry.x = entry->x;		// record the entry point
			fscb->m_nf_entry.y = entry->y;
		}
		fscb->m_nf_acc += dist;				// this records how far (into, total) this non-fuel block we are

		_type fl = fscb->m_fp->m_prevPoint->m_flameLength;

		if (fscb->m_nf_acc > (fl * 1.5 * fscb->grid_per_meter)) {	// grid resolution has already been inverted
			fscb->m_nf_status = FP_FLAG_NOFUEL;
			return 0;				// hit a non-fuel that was too "wide" that we can't breach so change where we stop and abort
		}
		return 1;					// we were able to breach the non-fuel - so far!
	}
	if (fscb->m_nf_acc > 0.0) {				// if we are coming out of a non-fuel area (and thus we could breach it)
		fscb->m_acc += fscb->m_nf_acc;			// extend the accumulator for how far we've travelled along by the same amount
		fscb->m_nf_acc = 0.0;				// mark that we've left the non-fuel that we were able to breach
		if (fscb->m_acc > fscb->m_orig_max) {		// if we have already gone further than the time step would allow, then we have to "push"/fudge this
			fscb->m_max = fscb->m_acc + s->breachAdvance();	// location just past the non-fuel boundary so it doesn't get marked as stopped, and...
			return 0;				// return that we've finished now
		}
	}
								// here, we are out of any encountered fuel break (or we haven't encountered any yet), and happily
								// marching along
	fscb->m_acc += dist;					// "accumulate" the distance travelled through this cell
	if (fscb->m_acc > fscb->m_orig_max)
		return 0;					// reached/passed the end of our ray/line so abort (and don't change the end of the line)
	return 1;						// go on to the next voxel/grid cell
}


template<class _type>
void FireFront<_type>::trackPointGrid(const FirePoint<_type> *actual_fp, FirePoint<_type> *fp, stepVoxelStart<_type> *svs) {
	ICWFGM_GridEngine *grid = Fire()->TimeStep()->m_scenario->m_scenario->m_gridEngine.get();

	NumericVariant nv;
	grid::AttributeValue av;
	XY_Point	utm_afp(*actual_fp),
				utm_fp(*fp);
	Fire()->TimeStep()->m_scenario->fromInternal(utm_afp);
	Fire()->TimeStep()->m_scenario->fromInternal(utm_fp);
	double utm_ax, utm_x, utm_ay, utm_y;
	grid->GetAttributeData(Fire()->TimeStep()->m_scenario->m_scenario->m_layerThread, utm_afp, Fire()->TimeStep()->m_time, WTimeSpan(0), CWFGM_FUELGRID_ATTRIBUTE_X_START, Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags, &nv, &av, nullptr);
	utm_ax = std::get<double>(nv);
	grid->GetAttributeData(Fire()->TimeStep()->m_scenario->m_scenario->m_layerThread, utm_fp, Fire()->TimeStep()->m_time, WTimeSpan(0), CWFGM_FUELGRID_ATTRIBUTE_X_START, Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags, &nv, &av, nullptr);
	utm_x = std::get<double>(nv);
	grid->GetAttributeData(Fire()->TimeStep()->m_scenario->m_scenario->m_layerThread, utm_afp, Fire()->TimeStep()->m_time, WTimeSpan(0), CWFGM_FUELGRID_ATTRIBUTE_Y_START, Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags, &nv, &av, nullptr);
	utm_ay = std::get<double>(nv);
	grid->GetAttributeData(Fire()->TimeStep()->m_scenario->m_scenario->m_layerThread, utm_fp, Fire()->TimeStep()->m_time, WTimeSpan(0), CWFGM_FUELGRID_ATTRIBUTE_Y_START, Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags, &nv, &av, nullptr);
	utm_y = std::get<double>(nv);

	if ((utm_ax != utm_x) || (utm_ay != utm_y)) {
		FireStateCallback<_type> fscb;				// only ray trace across the grid if we've moved to a new grid cell
		fscb.m_fs = this;
		fscb.m_fp = fp;
		fscb.m_grid = grid;
		fscb.grid_per_meter = svs->grid_per_meter;
		fscb.m_acc = 0.0;
		fscb.m_nf_acc = 0.0;
		fscb.m_nf_status = FP_FLAG_NORMAL;

		fscb.m_orig_max = fscb.m_max = actual_fp->DistanceTo(*fp);

		const XYZPointType	start(actual_fp->x, actual_fp->y, 0.0),
							end(fp->x, fp->y, 0.0);
		XYPointType s(utm_ax, utm_ay);
		Fire()->TimeStep()->m_scenario->toInternal(s);
		_type scale(1.0);
		Fire()->TimeStep()->m_scenario->gridToInternal1D(scale);
		const XYZPointType	offset(s.x - start.x, s.y - start.y, 0.0),
							step(scale, scale, scale);

#ifdef _DEBUG
		double dscale = scale;
		double gpm = fscb.grid_per_meter;
		XY_Point o(offset.x, offset.y);
		XY_Point af(actual_fp->x, actual_fp->y);
		XY_Point f(fp->x, fp->y);
#endif

		RayParmsTempl<_type> parms;
		parms.start = start;
		parms.path = end;
		parms.m_path_valid = true;
		parms.offset = offset;
		parms.step_size = step;
		parms.bounds_max_z = 1.0;
		parms.fcn = ::stepVoxelCallback<_type>;
		parms.parameter = &fscb;
		parms.DX = fp->m_ellipse_ros.x;		// these variables were set in AdvanceFire() to be the DX and DY at this point
		parms.DY = fp->m_ellipse_ros.y;
		parms.m_vertical = false;
		parms.RayTrace();

		if (fscb.m_nf_status) {					// ray tracing found something to stop against
			if (svs->use_lock)
				svs->lock_self.Lock();

			SetPoint(fp, fscb.m_nf_entry);

			fp->m_status = fscb.m_nf_status;

			if (svs->use_lock)
				svs->lock_self.Unlock();
		} else {
			if (fscb.m_orig_max != fscb.m_max) {		// this will happen when we breach a non-fuel area successfully
				XYPointType new_loc = *fp;
				new_loc.ScaleXY(fscb.m_max / fscb.m_orig_max, *fp->m_prevPoint);
				if (svs->use_lock)
					svs->lock_self.Lock();

				SetPoint(fp, new_loc);
				fp->m_successful_breach = 1;
				if (svs->use_lock)
					svs->lock_self.Unlock();
			}
		}
	}
}


#define QUEUE_UP_GRID	64

template<class _type>
void FireFront<_type>::TrackFireGrid() {
	stepVoxelStart<_type> svs;
	svs.self = this;
	svs.grid_per_meter = 1.0;
	Fire()->TimeStep()->m_scenario->toInternal1D(svs.grid_per_meter);

	if ((NumPoints() > QUEUE_UP_GRID) && (Fire()->TimeStep()->m_scenario->m_pool)) {
		svs.use_lock = true;

		std::vector<FirePoint<_type>*> &fp_array = Fire()->TimeStep()->m_scenario->m_omp_fp_array;
		std::int32_t i = 0, num_pts = NumPoints();
		if (fp_array.size() < num_pts)
			fp_array.resize(num_pts);
		FirePoint<_type> *fp1 = LH_Head();
		while (fp1->LN_Succ()) {
			if (fp1->m_status == FP_FLAG_NORMAL) {
				fp_array[i++] = fp1;
			}
			fp1 = fp1->LN_Succ();
		}
		num_pts = i;
		int thread_id = (Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY))) ? -1 : -2;
		#pragma omp parallel for num_threads(Fire()->TimeStep()->m_scenario->m_scenario->m_threadingNumProcessors) firstprivate(thread_id)
		for (i = 0; i < num_pts; i++) {
			if (thread_id == -1) {
				thread_id = omp_get_thread_num();
				CWorkerThread::native_handle_type thread = CWorkerThreadPool::GetCurrentThread();
				CWorkerThreadPool::SetThreadAffinityToMask(thread, thread_id);
			}
			FirePoint<_type> *curr = fp_array[i];

			if (curr->m_status == FP_FLAG_NORMAL) {
					trackPointGrid(curr->m_prevPoint, curr, &svs);
			}
		}

	} else { 
		svs.use_lock = false;
		FirePoint<_type> *curr = LH_Head();
		while (curr->LN_Succ()) {

			if (curr->m_status == FP_FLAG_NORMAL) {
				trackPointGrid(curr->m_prevPoint, curr, &svs);
			}
			curr = curr->LN_Succ();
		}
	}
}


template<class _type>
void FireFront<_type>::trackPointPrevFire(const FirePoint<_type> *actual_fp, FirePoint<_type> *fp, struct stepVoxelStart<_type> *svs) {

    #ifdef _DEBUG
	XYPointType _afp(*actual_fp), _fp(*fp);
    #endif

	XYLineType path(*actual_fp, *fp);						// create a line between the previous location and this one (which may have been shortened after hitting a grid boundary)
	RefList<XYPolyNodeType, XYPolyRefType> list;
	ScenarioFire<_type> *sf;
	const ScenarioTimeStep<_type> *sts;						// The purpose behind this is to avoid a fast burning fire from overtaking a slow-burning fire (which is reasonable when the head of a fire
	XYPolyRefType *node;								// catches up with the tail of another fire) AND then "eat" into the other fire's area that it had already burned in an unreasonable manner.
	XYPolyNodeType *other_fp;

	sts = Fire()->LN_CalcPred()->TimeStep();
	if (sts) {
		sf = sts->m_fires.LH_Head();
		while (sf->LN_Succ()) {
			if (sf->FastCollisionTest(path, 0.0))
				sf->IntersectionSet(path, list, 0, actual_fp);
			while (list.GetCount() > 1)
				delete list.RemTail();
			sf = sf->LN_Succ();
		}
		if (list.GetCount()) {
			node = list.RemHead();
			other_fp = node->LN_Ptr();
			if (svs->use_lock)
				svs->lock_self.Lock();

			SetPoint(fp, node->intersection_());
			fp->m_status = FP_FLAG_FIRE;
			if (svs->use_lock)
				svs->lock_self.Unlock();
			delete node;

			while ((node = list.RemHead()) != NULL)
				delete node;
		}
	}
}


template<class _type>
std::int32_t FireFront<_type>::pointInsideVectors(const XYPointType& pt) const {
	return Fire()->TimeStep()->pointInsideVectors(pt);
}



template<class _type>
std::int32_t ScenarioTimeStep<_type>::pointInsideVectors(const XYPointType &pt) const {
	std::int32_t inside = 0;
	std::uint32_t ii, ii_cnt;
	if (m_vectorBreaksLL)
		ii_cnt = (std::uint32_t)m_vectorBreaksLL->size();
	else
		ii_cnt = 0;
	for (ii = 0; ii < ii_cnt; ii++) {
		auto v = m_vectorBreaksLL->at(ii);
		if (v->box.PointInside(pt, RECT_POINTINSIDE_ALL_BORDERS))
			if (v->FastCollisionTest(pt, 0.0)) {
				inside = v->PointInArea(pt);
				if (inside)
					return inside;
			}
	}

	auto p = m_staticVectorBreaksLL.LH_Head();
	while (p->LN_Succ()) {
		const XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>* v = p->LN_Ptr();
		if (v->FastCollisionTest(pt, 0.0)) {
			inside = v->PointInArea(pt);
			if (inside)
				return inside;
		}
		p = p->LN_Succ();
	}

#if defined(_DEBUG) || defined(NO_TIMESTEP_CACHE)
	ii_cnt = m_scenario->StaticVectorBreakCount();
	for (ii = 0; ii < ii_cnt; ii++) {
		auto v = m_scenario->StaticVectorBreak(ii);
		if (v->box.PointInside(pt, RECT_POINTINSIDE_ALL_BORDERS)) {
			auto p = v->LH_Head();
			while (p->LN_Succ()) {
				if (p->Participates(m_time)) {
					auto pn = m_staticVectorBreaksLL.FindPtr(p);
					inside = p->PointInArea(pt);
					if (inside) {
						weak_assert(false);
						return inside;
					}
				}
				else {
					weak_assert(!p->FastCollisionTest(pt, 0.0));
				}
				p = p->LN_Succ();
			}
		}
	}
#endif

	return inside;
}


template<class _type>
void FireFront<_type>::trackPointVector(const FirePoint<_type> *actual_fp, FirePoint<_type> *fp, stepVoxelStart<_type> *svs) {

	XYLineType path(*actual_fp, *fp);					// create a line between the previous location and this one (which may have been shortened after hitting a grid boundary)
	XYRectangleType rect(path);

	RefList<XYPolyNodeType, XYPolyRefType> list;

	ScenarioFire<_type> *sf;
	bool after = false;
	sf = Fire()->TimeStep()->m_fires.LH_Head();			// here, we are testing against other fires...we pull back points which intersect with fires that are larger
	while (sf->LN_Succ()) {								// than 'this' (or are before us on the list if the areas happen to be the same)
		if (sf == m_fire) {
			after = true;								// ff is now 'after' this on the list
		} else {	// this would be considered a self-intersection and we deal with this elsewhere
			bool test = false;
			if (after) {
				if (m_fire->m_fireArea < sf->m_fireArea)
					test = true;
			} else {
				if (m_fire->m_fireArea <= sf->m_fireArea)
					test = true;
			}
			if (test) {
				if (sf->FastCollisionTest(rect, 0.0))
					sf->IntersectionSet(path, list);
				while (list.GetCount() > 1)
					delete list.RemTail();
			}
		}
		sf = sf->LN_Succ();
	}

	if (list.GetCount()) {
		XYPolyRefType *node = list.RemHead();
		path.p2 = node->intersection_();
		rect = XYRectangleType(path);
		if (svs->use_lock)
			svs->lock_self.Lock();

		SetPoint(fp, node->intersection_());

		fp->m_status = FP_FLAG_FIRE;
		if (svs->use_lock)
			svs->lock_self.Unlock();
		delete node;

		weak_assert(!list.GetCount());
		while ((node = list.RemHead()) != NULL)
			delete node;
	}

	Scenario<_type> *s = Fire()->TimeStep()->m_scenario;
	if (s->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_BREACHING)) {


	#ifdef _DEBUG
		{
			weak_assert(list.IsEmpty());
			auto rn = list.LH_Head();
			while (rn->LN_Succ()) {
				if (rn->LN_Pred()->LN_Pred()) {
					_type n_ds = rn->dist_sqr_();
					_type p_ds = rn->LN_Pred()->dist_sqr_();
					weak_assert(n_ds >= p_ds);// making sure things are in order
				}
				rn = rn->LN_Succ();
			}
		}
	#endif

    #ifdef DEBUG
		bool disable_quick;
    #endif

		XYLineType max_path(path);
		double fl = actual_fp->m_flameLength;

		_type dist_skip = fl * 1.525 * svs->grid_per_meter;
																	// this is the absolute maximum path that we will let a point travel - that is the length we think it will, plus the length of the flame, beyond that
		max_path.ChangeLength(path.Length() + dist_skip);			// we are simply pushing things too far for that given time step

		std::uint32_t ii, ii_cnt;
		if (Fire()->TimeStep()->m_vectorBreaksLL)
			ii_cnt = (std::uint32_t)Fire()->TimeStep()->m_vectorBreaksLL->size();
		else
			ii_cnt = 0;

		for (ii = 0; ii < ii_cnt; ii++)
			if (Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->box.Intersects(rect))
				if (Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->FastCollisionTest(rect, 0.0))
					Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->IntersectionSet(max_path, list);

		auto p = Fire()->TimeStep()->m_staticVectorBreaksLL.LH_Head();
		while (p->LN_Succ()) {
			const XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>* v = p->LN_Ptr();
			if (v->FastCollisionTest(rect, 0.0))
				v->IntersectionSet(max_path, list);
			p = p->LN_Succ();
		}

		if (list.GetCount()) {							// this is where breaching vector breaks should go.  When this is done, the raytracing of grid cells likely (shouldn't) be needed any more

			_type m_acc = 0.0, m_nf_acc = 0.0;
			_type m_orig_max = path.Length() + 0.00005;			// we do a bit of fudging to deal with rounding errors
			XYPointType m_nf_entry;
			XYPolyRefType *nn, *node = list.LH_Head();
			if (node->dist_sqr_() > 0.0) {
				weak_assert(!node->intersection_().Equals(*actual_fp));		// our breaching implementation here is kept absolutely consistent with the grid breaching
				node = new XYPolyRefType;
				node->set_intersection_(*actual_fp);
				node->LN_Ptr(nullptr);
				node->set_dist_sqr_(0.0);
				list.AddHead(node);

			}
			node = list.LH_Tail();
			if (!node->intersection_().Equals(max_path.p2)) {
				node = new XYPolyRefType;
				node->set_intersection_(max_path.p2);
				node->LN_Ptr(nullptr);
				node->set_dist_sqr_(max_path.LengthSqr());
				list.AddTail(node);
			}

														// now we have complete line segments for everything along our path - including where we start from and where we think we may end up at
			node = list.LH_Head();
			nn = node->LN_Succ();
			XYPointType mid;
			std::int32_t inside = 0;
			while (nn->LN_Succ()) {
				mid = node->PointBetweenIntersections(nn);
				inside = pointInsideVectors(mid);

				if (inside) {						// if we are in a break...
					if (m_nf_acc == 0.0) {				// if we're at the start, then record it
						m_nf_entry = node->intersection_();
					}

					XYPointType i0(node->intersection_()), i1(nn->intersection_());
					m_nf_acc += i0.DistanceTo(i1);

					if (m_nf_acc > (fl * 1.5 * svs->grid_per_meter)) {
						if (svs->use_lock)
							svs->lock_self.Lock();		// if we couldn't breach this fire, then record the point stopping moving at where we entered the fire break, then break out of the loop

						SetPoint(fp, m_nf_entry);
						fp->m_status = FP_FLAG_VECTOR;
						if (svs->use_lock)
							svs->lock_self.Unlock();
						break;
					}
				} else {				// so we're not in a break
					if (m_nf_acc > 0.0) {				// did we just leave a break? 
						
						m_acc += m_nf_acc;				// yep, so record into our main accumulator how long that break was, that we just breached
						m_nf_acc = 0.0;
						if (m_acc > m_orig_max) {
							max_path.ChangeLength(m_acc + Fire()->TimeStep()->m_scenario->breachAdvance());	// the point as just outside that break
							if (svs->use_lock)
								svs->lock_self.Lock();

							SetPoint(fp, max_path.p2);
							fp->m_successful_breach = 1;

							if (svs->use_lock)
								svs->lock_self.Unlock();
							break;
						}
					}

					XYPointType i0(node->intersection_()), i1(nn->intersection_());
					m_acc += i0.DistanceTo(i1);

					if (m_acc >= m_orig_max)			// if we're done then we're done
						break;
				}

				node = nn;
				nn = nn->LN_Succ();
			}
			if (!nn->LN_Succ()) {						// if we got here, then what does that mean?...that means that we travelled the length of the line, plus the flamelength*1.5 for breaching
				weak_assert(false);						// so we should have either been stopped by a fire break or left all the fire breaks to continue burning - i.e. exited the loop with one
			}								// of the above 'break' statements - so this is a bad case that needs to be investigated.
		}
	} else {
		std::uint32_t ii, ii_cnt;
		if (Fire()->TimeStep()->m_vectorBreaksLL)
			ii_cnt = (std::uint32_t)Fire()->TimeStep()->m_vectorBreaksLL->size();
		else
			ii_cnt = 0;
		for (ii = 0; ii < ii_cnt; ii++) {
			if (Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->box.Intersects(rect))
				if (Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->FastCollisionTest(rect, 0.0))
					Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->IntersectionSet(path, list);
			while (list.GetCount() > 1)
				delete list.RemTail();
		}

		auto p = Fire()->TimeStep()->m_staticVectorBreaksLL.LH_Head();
		while (p->LN_Succ()) {
			const XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>* v = p->LN_Ptr();
			if (v->FastCollisionTest(rect, 0.0))
				v->IntersectionSet(path, list);
			while (list.GetCount() > 1)
				delete list.RemTail();
			p = p->LN_Succ();
		}

		if (list.GetCount()) {
			XYPolyRefType *node = list.RemHead();
			if (svs->use_lock)
				svs->lock_self.Lock();

			SetPoint(fp, node->intersection_());
			fp->m_status = FP_FLAG_VECTOR;
			if (svs->use_lock)
				svs->lock_self.Unlock();
			delete node;
		} else if ((Fire()->TimeStep()->m_scenario->m_scenario->m_perimeterSpacing > 0.0) && (fp->m_status == FP_FLAG_NORMAL)) {					// see if the point (that's still active) is "too close" to another vertex
		 // first test against vector breaks, then we'll need to test against other fire perimeters
			RefList<XYPolyNodeType, XYPolyDistType> plist;
			weak_assert(list.IsEmpty());

			_type fudge = Fire()->TimeStep()->m_scenario->m_scenario->m_perimeterSpacing;
			Fire()->TimeStep()->m_scenario->toInternal1D(fudge);
			std::uint32_t ii, ii_cnt;
			if (Fire()->TimeStep()->m_vectorBreaksLL)
				ii_cnt = (std::uint32_t)Fire()->TimeStep()->m_vectorBreaksLL->size();
			else
				ii_cnt = 0;
			for (ii = 0; ii < ii_cnt; ii++) {
				if (Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->box.Intersects(rect))
					if (Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->FastCollisionTest(rect, 0.0)) {
						Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->PointSet(*fp, fudge, true, plist, fp);
						Fire()->TimeStep()->m_vectorBreaksLL->at(ii)->EdgeSet(*fp, fudge, true, list, fp);
					}
				while (plist.GetCount() > 1)
					delete plist.RemTail();
				while (list.GetCount() > 1)
					delete list.RemTail();
			}

			auto p = Fire()->TimeStep()->m_staticVectorBreaksLL.LH_Head();
			while (p->LN_Succ()) {
				const XY_PolyLL_Templ<XY_PolyLLNode<_type>, _type>* v = p->LN_Ptr();
				if (v->FastCollisionTest(rect, 0.0)) {
					v->PointSet(*fp, fudge, true, plist, fp);
					v->EdgeSet(*fp, fudge, true, list, fp);
				}
				while (plist.GetCount() > 1)
					delete plist.RemTail();
				while (list.GetCount() > 1)
					delete list.RemTail();
				p = p->LN_Succ();
			}

			if (plist.IsEmpty()) {
				XYPolyRefType *node = list.RemHead();
				if (node) {
					if (svs->use_lock)
						svs->lock_self.Lock();

					SetPoint(fp, *node->LN_Ptr());
					fp->m_status = FP_FLAG_VECTOR;

					if (svs->use_lock)
						svs->lock_self.Unlock();
					delete node;
				}
			} else {
				XYPolyDistType *node = plist.RemHead();
				if (svs->use_lock)
					svs->lock_self.Lock();

				SetPoint(fp, *node->LN_Ptr());
				fp->m_status = FP_FLAG_VECTOR;

				if (svs->use_lock)
					svs->lock_self.Unlock();
				delete node;

				XYPolyRefType *node1 = list.RemHead();
				if (node1)
					delete node1;
			}

			weak_assert(list.IsEmpty());
			weak_assert(plist.IsEmpty());

			if (fp->m_status == FP_FLAG_NORMAL) {
				ScenarioFire<_type> *sf;
				bool after = false;
				sf = Fire()->TimeStep()->m_fires.LH_Head();				// here, we are testing against other fires...we pull back points which intersect with fires that are larger
				while (sf->LN_Succ()) {									// than 'this' (or are before us on the list if the areas happen to be the same)
					bool test = false;
					if (sf == m_fire) {
						after = true;									// ff is now 'after' this on the list
						test = true;
					}
					else {		// this would be considered a self-intersection and we deal with this elsewhere
						if (after) {
							if (m_fire->m_fireArea < sf->m_fireArea)
								test = true;
						} else {
							if (m_fire->m_fireArea <= sf->m_fireArea)
								test = true;
						}
					}
					if (test) {
						if (sf == m_fire) {
							if (svs->use_lock)
								svs->lock_self.Lock();
							if (sf->FastCollisionTest(*fp, 0.0)) {
								sf->PointSet(*fp, fudge, true, plist, fp);
								sf->EdgeSet(*fp, fudge, true, list, fp);
							}
							if (svs->use_lock)
								svs->lock_self.Unlock();
						} else if (sf->FastCollisionTest(rect, 0.0)) {
							sf->PointSet(*fp, fudge, true, plist, fp);
							sf->EdgeSet(*fp, fudge, true, list, fp);
						}
						while (plist.GetCount() > 1)
							delete plist.RemTail();
						while (list.GetCount() > 1)
							delete list.RemTail();
					}
					sf = sf->LN_Succ();
				}
				if (plist.IsEmpty()) {
					XYPolyRefType *node = list.RemHead();
					if (node) {
						if (svs->use_lock)
							svs->lock_self.Lock();

						SetPoint(fp, *node->LN_Ptr());
						fp->m_status = FP_FLAG_FIRE;

						if (svs->use_lock)
							svs->lock_self.Unlock();
						delete node;
					}
				} else {
					XYPolyDistType *node = plist.RemHead();
					if (svs->use_lock)
						svs->lock_self.Lock();

					SetPoint(fp, *node->LN_Ptr());
					fp->m_status = FP_FLAG_FIRE;

					if (svs->use_lock)
						svs->lock_self.Unlock();
					delete node;
				}
			}
		}
	}
	XYPolyRefType *node;
	while ((node = list.RemHead()) != nullptr)
		delete node;
}


#define QUEUE_UP_VECTOR		64


template<class _type>
void FireFront<_type>::TrackFireVector() {
	stepVoxelStart<_type> svs;
	svs.self = this;
	svs.grid_per_meter = 1.0;
	Fire()->TimeStep()->m_scenario->toInternal1D(svs.grid_per_meter);

	if ((NumPoints() > QUEUE_UP_VECTOR) && (Fire()->TimeStep()->m_scenario->m_pool)) {
		svs.use_lock = true;

		std::vector<FirePoint<_type>*> &fp_array = Fire()->TimeStep()->m_scenario->m_omp_fp_array;
		std::int32_t i = 0, num_pts = NumPoints();
		if (fp_array.size() < num_pts)
			fp_array.resize(num_pts);
		FirePoint<_type> *fp1 = LH_Head();
		while (fp1->LN_Succ()) {
			if (!fp1->Equals(*fp1->m_prevPoint))
				fp_array[i++] = fp1;
			fp1 = fp1->LN_Succ();
		}
		num_pts = i;
		int thread_id = (Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY))) ? -1 : -2;
		#pragma omp parallel for num_threads(Fire()->TimeStep()->m_scenario->m_scenario->m_threadingNumProcessors) firstprivate(thread_id)
		for (i = 0; i < num_pts; i++) {
			if (thread_id == -1) {
				thread_id = omp_get_thread_num();
				CWorkerThread::native_handle_type thread = CWorkerThreadPool::GetCurrentThread();
				CWorkerThreadPool::SetThreadAffinityToMask(thread, thread_id);
			}
			FirePoint<_type> *curr = fp_array[i];
			weak_assert(!curr->Equals(*curr->m_prevPoint));
			trackPointPrevFire(curr->m_prevPoint, curr, &svs);
		}

		thread_id = (Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_FORCE_AFFINITY))) ? -1 : -2;
		#pragma omp parallel for num_threads(Fire()->TimeStep()->m_scenario->m_scenario->m_threadingNumProcessors) firstprivate(thread_id)
		for (i = 0; i < num_pts; i++) {
			if (thread_id == -1) {
				thread_id = omp_get_thread_num();
				CWorkerThread::native_handle_type thread = CWorkerThreadPool::GetCurrentThread();
				CWorkerThreadPool::SetThreadAffinityToMask(thread, thread_id);
			}
			FirePoint<_type> *curr = fp_array[i];

			if (!curr->Equals(*curr->m_prevPoint)) {
				trackPointVector(curr->m_prevPoint, curr, &svs);
			}
		}

	} else {
		svs.use_lock = false;
		FirePoint<_type> *curr = LH_Head();
		while (curr->LN_Succ()) {
			if (!curr->Equals(*curr->m_prevPoint))
				trackPointPrevFire(curr->m_prevPoint, curr, &svs);
			curr = curr->LN_Succ();
		}
		curr = LH_Head();
		while (curr->LN_Succ()) {
			if (!curr->Equals(*curr->m_prevPoint)) {
				trackPointVector(curr->m_prevPoint, curr, &svs);
			}
			curr = curr->LN_Succ();
		}
	}
}


#include "InstantiateClasses.cpp"
template struct stepVoxelStart<fireengine_float_type>;
template struct FireStateCallback<fireengine_float_type>;
