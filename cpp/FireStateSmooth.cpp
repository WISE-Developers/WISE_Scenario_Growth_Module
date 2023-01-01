/**
 * WISE_Scenario_Growth_Module: FireStateSmooth.cpp
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
#include "ScenarioTimeStep.h"
#include "scenario.h"
#include "FireEngine_ext.h"


template<class _type>
_type FireFront<_type>::adjustAngle(bool convex, _type angle) {
	if ((convex) && (angle > DEGREE_TO_RADIAN(225.0))) {
		angle -= DEGREE_TO_RADIAN(225.0);
		angle /= 1.625;
		angle += DEGREE_TO_RADIAN(225.0);
	}
	return angle;
}


template<class _type>
void FireFront<_type>::AddPoints() {
	if (NumPoints() < 3)
		return;
	const FirePoint<_type>	*curr,
							*prev,
							*next,
							*last_pt;
	FirePoint<_type>		*new_pt;
	_type					angle, dist;					// basic seeds

	next = LH_Head();
	curr = LH_Tail();
	prev = curr->LN_Pred();

	std::uint16_t		cnt_next, cnt_prev;

	_type perimeterResolution;
	bool convex = (Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT))) ? true : false;
	if (Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC)) {
		_type area = Fire()->InitArea() / 10000.0;
		Fire()->TimeStep()->m_scenario->fromInternal2D(area);
		perimeterResolution = Fire()->TimeStep()->m_scenario->m_scenario->perimeterResolution(area);	// dealing with scale in the call, so don't need to do it here
	} else
		perimeterResolution = Fire()->TimeStep()->m_scenario->m_scenario->perimeterResolution(0.0);
	Fire()->TimeStep()->m_scenario->gridToInternal1D(perimeterResolution);

	while (next->LN_Succ()) {
		if ((prev->m_status == FP_FLAG_NORMAL) || (curr->m_status == FP_FLAG_NORMAL) || (next->m_status == FP_FLAG_NORMAL)) {

			angle = curr->AngleBetween(*prev, *next);
			angle = adjustAngle(convex, angle);
			_type dist_factor, sin_angle = sin(angle * 0.5);

			cnt_prev = 0;
			if ((prev->m_status == FP_FLAG_NORMAL) || (curr->m_status == FP_FLAG_NORMAL)) {
				dist = curr->DistanceTo(*prev);
				dist_factor = dist / perimeterResolution;
				last_pt = prev;

				if (dist_factor > 2.0) {
					equiDistantPoints(prev, curr, dist_factor);
					last_pt = curr->LN_Pred();
					dist = curr->DistanceTo(*last_pt);
					dist_factor = dist / perimeterResolution;
				}

				while ((cnt_prev < 2) && (dist_factor > 0.001) && (sin_angle < dist_factor)) {					// logic keeps points dropping to under 1/1000'th of a grid cell.  The new logic around decimate forces a
					XYPointType loc = last_pt->PointBetween(*curr);				// hard minimum based on the user preferences - which typically won't be met (because of the existing logic)
																				// but may be important for very small grid resolutions

					new_pt = new FirePoint<_type>(loc);
					InsertPoint(new_pt, (FirePoint<_type>*)last_pt);
					last_pt = new_pt;
					dist_factor *= 0.5;
					cnt_prev++;
				}
			}

			cnt_prev = 0;						// this var is re-used to record that we've used the (if dist_factor > 2.0) part of the code below
			cnt_next = 0;
			if ((curr->m_status == FP_FLAG_NORMAL) || (next->m_status == FP_FLAG_NORMAL)) {
				dist = curr->DistanceTo(*next);
				dist_factor = dist / perimeterResolution;

				if (dist_factor > 2.0) {
					equiDistantPoints(curr, next, dist_factor);
					cnt_prev = 1;
					last_pt = curr->LN_Succ();
					dist = curr->DistanceTo(*last_pt);
					dist_factor = dist / perimeterResolution;
				} else
					last_pt = next;

				while ((cnt_next < 3) && (dist_factor > 0.001) && (sin_angle < dist_factor)) {					// logic keeps points dropping to under 1/1000'th of a grid cell.  The new logic around decimate forces a
					XYPointType loc = last_pt->PointBetween(*curr);				// hard minimum based on the user preferences - which typically won't be met (because of the existing logic)
																				// but may be important for very small grid resolutions

					new_pt = new FirePoint<_type>(loc);
					InsertPoint(new_pt, (FirePoint<_type>*)curr);
					last_pt = new_pt;
					dist_factor *= 0.5;
					cnt_next++;
				}
			}
			if ((cnt_next) || (cnt_prev))			// if we added even one point between curr and next...
				prev = next->LN_PredWrap();			// then jump ahead a bit 'cause we can
			else
				prev = curr;

		} else
			prev = curr;
													// without the look-ahead if statement for (dist_factor > 2.0), then
		curr = next;								// this kind of stepping forward can still cause issues with parts of the line being greater than maxline_threshold 'cause of how we
		next = next->LN_Succ();						// divide up the line consecutively closer to the original "curr" and the original "next" (or prev)

	}
}


template<class _type>
bool FireFront<_type>::simplify(FirePoint<_type> *prev, FirePoint<_type> *curr, FirePoint<_type> *succ, const _type perimeterSpacing, const _type perimeterResolution, const bool convex, const _type n_pr, const _type s_pr, FirePoint<_type> **sel, _type *sel_dist, _type *sel_angle) {
	weak_assert(curr->LN_PredWrap() == prev);
	weak_assert(curr->LN_SuccWrap() == succ);
	const _type		s_dist = curr->DistanceTo(*succ),
					p_dist = curr->DistanceTo(*prev),
					n_dist = succ->DistanceTo(*prev);
	bool select_delete = false, retval = true;
	_type sin_angle;
	const _type dist = min(s_dist, p_dist);
	if (dist < perimeterSpacing) {										// if the distance is simply too short, then don't bother with all the rest, flag it as a point we can remove
		select_delete = true;
		_type angle = curr->AngleBetween(*prev, *succ);
		angle = adjustAngle(convex, angle);
		sin_angle = sin(angle * 0.5);
	}
	else {
		if (((s_dist + p_dist) < n_pr) || (n_dist < s_pr)) {			// check to see if this point could be a candidate for removal - don't want to remove a point that
																		// would just be re-introduced in AddPoints()
			_type angle = curr->AngleBetween(*prev, *succ);
			angle = adjustAngle(convex, angle);
			sin_angle = sin(angle * 0.5);
			if (sin_angle > (dist / n_pr)) {							// if the ratio between the angle and distance is suitable...
				const _type n__dist_factor = n_dist / perimeterResolution;
				_type n__angle = prev->AngleBetween(*prev->LN_PredWrap(), *succ);
				n__angle = adjustAngle(convex, n__angle);
				_type n__sin_angle = sin(n__angle * 0.5);
				if (n__sin_angle > n__dist_factor) {							// the prior and latter logic tests to see if we'd re-introduce a point if we removed it
					n__angle = succ->AngleBetween(*prev, *succ->LN_SuccWrap());	// if we'd re-introduce the point, then there's no point in removing this one
					n__angle = adjustAngle(convex, n__angle);
					n__sin_angle = sin(n__angle * 0.5);
					if (n__sin_angle > n__dist_factor)
						select_delete = true;
				}
			}
		}
		else
			retval = false;
	}
	if (select_delete) {
		if ((!(*sel)) ||
			(dist < (*sel_dist)) ||
			((dist == (*sel_dist)) && (sin_angle > (*sel_angle)))) {						// pick the point that meets the curvature criteria, and that has the shortest edge

			*sel = curr;
			*sel_dist = dist;
			*sel_angle = sin_angle;
		}
	}
	return retval;		// if this simply isn't a candidate, then say that we can abort
}


template<class _type>
std::uint32_t FireFront<_type>::Simplify() {
	std::uint32_t cnt = 0;
	if (NumPoints() <= 3)
		return cnt;

	if (PreviousMinimumROSRatio(true) < 0.90)									// don't bother clipping out any points during the acceleration phase (when the fire is very small), let
		return cnt;																// it start growing first

	_type max_dist = 0.0;
	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {														// this loop is necessary.  The original intent was to place a hard minimum in distances between 2 points
		FirePoint<_type> *next = fp->LN_SuccWrap();								// (which we can keep), but for fires that are just starting to grow, we need to account for the maximum
		_type l_dist = fp->DistanceTo(*next);									// edge length still being less than the perimeter resolution
		if (l_dist > max_dist)
			max_dist = l_dist;
		fp = fp->LN_Succ();
	}

	_type perimeterResolution, pr;
	_type perimeterSpacing = Fire()->TimeStep()->m_scenario->m_scenario->m_perimeterSpacing;
	const bool convex = (Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT))) ? true : false;
	if (Fire()->TimeStep()->m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC)) {
		_type area = Fire()->InitArea() / 10000.0;
		Fire()->TimeStep()->m_scenario->fromInternal2D(area);
		perimeterResolution = Fire()->TimeStep()->m_scenario->m_scenario->perimeterResolution(area);	// dealing with scale in the call, so don't need to do it here
	} else
		perimeterResolution = Fire()->TimeStep()->m_scenario->m_scenario->perimeterResolution(0.0);
	Fire()->TimeStep()->m_scenario->gridToInternal1D(perimeterResolution);

	if (max_dist < perimeterResolution)
		pr = max_dist;											// take the lesser of the maximum edge length, and the perimeter resolution
	else
		pr = perimeterResolution;

	const _type s_pr = pr / 2.5;
	const _type n_pr = pr / sqrt(2.0);
	FirePoint<_type> *c = LH_Head();
	_type	d_angle, d_dist, sin_angle;
	FirePoint<_type> *d;
	while (c->LN_Succ()) {
		d = nullptr;

		FirePoint<_type> *succ = c->LN_SuccWrap();
		FirePoint<_type> *prev = c->LN_PredWrap();
		if ((c->m_status == FP_FLAG_NORMAL) && ((prev->m_status == FP_FLAG_NORMAL) || (succ->m_status == FP_FLAG_NORMAL))) {									// can only eliminate active points
			FirePoint<_type> *curr = c;

			// search backwards
			while (curr->m_status == FP_FLAG_NORMAL) {							// the next 2 loops search forward and back for the most ideal candidate for a vertex to be removed.
				prev = curr->LN_PredWrap();										// I'm trying to avoid any concerns about where we start in the set of vertices, or the order
				if (prev == c)
					break;
				if (!simplify(prev, curr, succ, perimeterSpacing, perimeterResolution, convex, n_pr, s_pr, &d, &d_dist, &d_angle))
					break;
				succ = curr;
				curr = prev;
			}

			curr = c;
			prev = curr->LN_PredWrap();
			while (curr->m_status == FP_FLAG_NORMAL) {							// if it's a possible candidate to remove...
				succ = curr->LN_SuccWrap();
				if (succ == c)
					break;
				if (!simplify(prev, curr, succ, perimeterSpacing, perimeterResolution, convex, n_pr, s_pr, &d, &d_dist, &d_angle))
					break;
				prev = curr;
				curr = succ;
			}

			if (d) {
				if (d == c) {													// if we're removing the iterator, then adjust the iterator
					if (c->LN_Pred()->LN_Pred())
						c = c->LN_Pred();
					else
						c = c->LN_Succ();
				}

				RemovePoint(d);
				delete d;
				cnt++;
				if (NumPoints() <= 3)
					return cnt;
			}
			else
				c = c->LN_Succ();
		}
		else
			c = c->LN_Succ();
	}

	return cnt;
}


template<class _type>
void FireFront<_type>::equiDistantPoints(const FirePoint<_type> *start, const FirePoint<_type> *end, _type dist_factor) {

	_type steps = ceil(dist_factor);
	std::uint32_t cnt_next, num = (std::uint32_t)steps;
	XYPointType delta = (*end - *start);
	delta /= steps;
	XYPointType loc = *start + delta;
	for (cnt_next = 1; cnt_next < num; cnt_next++, loc += delta) {
		FirePoint<_type> *new_pt = new FirePoint<_type>(loc);

		InsertPoint(new_pt, (FirePoint<_type>*)start);
		start = new_pt;
	}
}


#include "InstantiateClasses.cpp"
