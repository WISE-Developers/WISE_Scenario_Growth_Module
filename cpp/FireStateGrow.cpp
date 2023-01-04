/**
 * WISE_Scenario_Growth_Module: FireStateGrow.cpp
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
#include <assert.h>
#include <cmath>
#include <omp.h>
#include "propsysreplacement.h"

#include "scenario.h"
#include "CWFGM_Scenario_Internal.h"
#include "FireEngine_ext.h"
#include "FuelCom_ext.h"
#include "CWFGM_Scenario.h"
#include "GridCom_ext.h"
#include "results.h"
#include "rectangles.h"
#include "raytrace.h"
#include "firepoint.h"
#include "ScenarioTimeStep.h"

#define QUEUE_UP	64

template<class _type>
void FirePoint<_type>::grow2D(const XYZPointType &p_pt, const XYZPointType &s_pt) {

	double	a = (m_fbp_ros + m_fbp_bros) * 0.5,		// have to convert from ROS, etc. to Gwyn's required a, b, c - this is the
			b = m_fbp_fros,					// same as in both the Win16 and Unix versions of FireBrand
			c = (m_fbp_ros - m_fbp_bros) * 0.5,
			a2 = a * a,
			b2 = b * b;
	_type		_x = s_pt.x - p_pt.x,				// the rest of this calculation is essentially the same as the Win16 version
				_y = s_pt.y - p_pt.y,				// of the elliptical fire growth model - to be subsequently replaced but
		cs,									// this should do for now.
		sn,
		xcsysn,
		xsnycs,
		denominator;
	::sincos(m_fbp_raz, &sn, &cs);
	xcsysn = _x * cs - _y * sn;
	xsnycs = _x * sn + _y * cs;
	denominator = sqrt(a2 * xcsysn * xcsysn + b2 * xsnycs * xsnycs);
	if (denominator > 0.0) {
		m_ellipse_ros.x = (b2 * cs * xsnycs - a2 * sn * xcsysn) / denominator + c * sn;
		m_ellipse_ros.y = (-b2 * sn * xsnycs - a2 * cs * xcsysn) / denominator + c * cs;
		weak_assert((m_ellipse_ros.x != 0.0) || (m_ellipse_ros.y != 0.0));
	} else {

		m_ellipse_ros.x = m_ellipse_ros.y = 0.0;
		m_fbp_ros_ratio = 1.0;
	}
}


template<class _type>
void FirePoint<_type>::grow3D(const class ScenarioTimeStep<_type> *timeStep, const XYZPointType &c_pt, const XYZPointType &p_pt, const XYZPointType &s_pt, _type aspect, const _type azimuth) {
	double a = (m_fbp_ros + m_fbp_bros) * 0.5;	// have to convert from ROS, etc. to Gwyn's required a, b, c - this is the
	double b = m_fbp_fros;				// same as in both the Win16 and Unix versions of FireBrand
	double c = (m_fbp_ros - m_fbp_bros) * 0.5;
	_type	cs;							// remember, RAZ is in COMPASS, not in CARTESIAN...
	_type	sn;
	_type	cs_a;						// ...but azimuth IS in CARTESIAN (important below)
	_type	sn_a;
	::sincos(m_fbp_raz, &sn, &cs);
	::sincos(azimuth, &sn_a, &cs_a);

	//The azimuth and aspect of curr are in the respective variable names
	//From these, we can determine the partial derivatives of F at curr
	//Azimuth in cartesian radians
	//Aspect is now in %age slope (decimal) (was before in radians)

	XYZPointType N;					// f is a unit (length) vector of the slope, N is the normal upward vector to the slope

	//This detects when the topography checkbox is checked.
	if ((timeStep->m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_TOPOGRAPHY)) && (aspect > 0.0)) {
							// if FBP-slope-calcs is active AND there is some slope...
							// This is important 'cause we shouldn't be disabling slope calc's in the FBP module, then using them here - have to be
							// consistent - it doesn't make sense to disable the FBP wse (wind speed equivalent) calc and then apply topography to
							// how the point should move on the terrain.
							// But you can go the other way (at least right now) - use FBP slope calc's, then use 2-D ellipse modelling - somehow
							// this seems to make more sense but maybe that should be reviewed too.
		XYZPointType f;

		f.x = cs_a;
		f.y = sn_a;
		f.z = aspect;

		const XYZPointType f1(-f.y, f.x, 0.0);	// f1 is a vector which is 90 degrees to the "up-slope" vector (in the XY_plane) with no up-slope ('cause it's normal
							// to the up-slope)
		N = f.CrossProduct(f1);			// these 2 lines are the same as the call to NormalVector() below but without moving the points to relative to origin
		N.Normalize();				// 'cause they already are!

		if (N.z < 0.0) {			// this normal vector is supposed to go "up" - stated in the paper and makes sense - I've oriented f1 to (it seems)
			N.x = 0.0 - N.x;		// always calculate N as up, but the check is really cheap to perform so I'll leave that here for now.
			N.y = 0.0 - N.y;
			N.z = 0.0 - N.z;
		}
	} else {
		N.x = N.y = 0.0;
		N.z = 1.0;				// if there's no slope, then the normal to the plane is straight up...
		aspect = 0.0;			// corrects for the "default / no values present" case that we can get from GridCOM
	}
	
	//Here, we approximate theta to be 2-dimensional, and to be in the direction of m_raz
	//We can also assume that the length of theta is 1
	XYZPointType theta;
	theta.x = sn;					// sn, cs are reversed here on purpose - remember that RAZ is in COMPASS
	theta.y = cs;
	if (aspect > 0.0)
		theta.z = (cs_a * theta.x + sn_a * theta.y) * aspect;
	else	theta.z = 0.0;
							// the idea behind the z value is as follows: we have a ray defined by 'f'.  That ray defines a plane, and N is the
							// normal to the plane - so 'f' goes straight "up" the plane.  Now we've got theta.  What I want is the 'z' value for
							// (theta.x, theta.y) on the plane defined by z.
							// we use the same logic to calculate r.z below.

	theta.Normalize();

	//Find the distance between pred and curr, and the distance between curr and succ (3-dimensions)
	//For calculation below
	_type plen = c_pt.XYZ_DistanceTo(p_pt);
	_type slen = c_pt.XYZ_DistanceTo(s_pt);	// plen and slen are effectively L in the paper, as defined as part of equation 27

	XYZPointType r;					// r is defined at the "unit tangent vector to the perimeter at" (x,y) (2nd page)
							// I haven't experimented with it, but I suspect the order of the points, as well as the fact that the vectors created
							// both "end" at the current point of interest.  I've drawn a few examples out on paper and it all seems reasonable.
	r.x = slen * (s_pt.x - c_pt.x) - plen * (p_pt.x - c_pt.x);
	r.y = slen * (s_pt.y - c_pt.y) - plen * (p_pt.y - c_pt.y);
	if (aspect > 0.0)
		r.z = (cs_a * r.x + sn_a * r.y) * aspect;
	else	r.z = 0.0;
	if ((r.x == 0.0) && (r.y == 0.0))
		r.z = 1.0;
	r.Normalize();					// this, I think, is equation #27 fixed (and adjusted because we have slope information which should in theory be more
							// accurate than the info defined by "attached" z values to other points since our elevation is pixel-ized and not
							// defined mathematically as a surface).  This is similar to just adding a couple vectors together and applying Z values,
							// but not quite the same.

	//These values are just simplifications of the calculations of the parameters in equations 19 and 20 below
	//They are also used for the cross product simplification of equation 30 (sinalpha) below.
	XYZPointType Ntheta(N.CrossProduct(theta));					// temporary for equations 29, 19, 20
	
	//Used to calculate cos(alpha) and sin(alpha)
	//Calculated using the cross product from Eq 28 of G. Richards model
	XYZPointType n(r.CrossProduct(N));							// G. Richards Eq. 28

	_type cosalpha = n.DotProduct(theta);						// G. Richards Eq. 29
	_type sinalpha = Ntheta.DotProduct(n);						// G. Richards Eq. 30
	_type divider = sqrt(a * a * cosalpha * cosalpha + b * b * sinalpha * sinalpha);

	if (divider != 0.0) {
		_type Xalpha = (a * a * cosalpha) / divider + c;				// G. Richards Eq. 8
		_type Yalpha = (b * b * sinalpha) / divider;					// G. Richards Eq. 9

	//Find the new location based on the above calculations

		m_ellipse_ros.x = Xalpha * theta.x + Yalpha * Ntheta.x;				// G. Richards Eq. 19
		m_ellipse_ros.y = Xalpha * theta.y + Yalpha * Ntheta.y;				// G. Richards Eq. 20
	} else {
		m_ellipse_ros.x = m_ellipse_ros.y = 0.0;
		m_fbp_ros_ratio = 1.0;
	}
}


template<class _type>
void FirePoint<_type>::Grow(const growVoxelParms<_type> *gvs, ICWFGM_Fuel *fuel) {
	double latitude = gvs->latitude;
	double longitude = gvs->longitude;
	WTimeSpan accel_dtime = gvs->accel_dtime;
	const ScenarioTimeStep<_type> *sts = gvs->self_fire_timestep;
	grid::TerrainValue elev_valid, terrain_valid;

	weak_assert(!m_status);

	const FirePoint<_type>	*pred = LN_PredWrap(),
							*succ = LN_SuccWrap();

	XYZPointType	c_pt(x, y, 0.0),
					p_pt(pred->x, pred->y, 0.0),
					s_pt(succ->x, succ->y, 0.0);

	double		aspect, azimuth;
	std::uint64_t flags = sts->m_scenario->m_scenario->m_optionFlags;

	HRESULT hr;

	CCWFGM_FuelOverrides overrides;
	sts->m_scenario->GetCorrectedFuel(c_pt, sts->m_time, fuel, overrides);

	XY_Point ppt, spt, cpt;
	ppt.x = p_pt.x;
	ppt.y = p_pt.y;
	spt.x = s_pt.x;
	spt.y = s_pt.y;
	cpt.x = c_pt.x;
	cpt.y = c_pt.y;
	sts->m_scenario->fromInternal(ppt);
	sts->m_scenario->fromInternal(spt);
	sts->m_scenario->fromInternal(cpt);
	double __z;
	hr = gvs->grid->GetElevationData(sts->m_scenario->m_scenario->m_layerThread, ppt, true, &__z, &aspect, &azimuth, &elev_valid, &terrain_valid, nullptr); p_pt.z = __z;
	hr = gvs->grid->GetElevationData(sts->m_scenario->m_scenario->m_layerThread, spt, true, &__z, &aspect, &azimuth, &elev_valid, &terrain_valid, nullptr); s_pt.z = __z;
	hr = gvs->grid->GetElevationData(sts->m_scenario->m_scenario->m_layerThread, cpt, true, &__z, &aspect, &azimuth, &elev_valid, &terrain_valid, nullptr); c_pt.z = __z;

	double fmc, ff;

	if (gvs->m_specifiedFMC >= 0.0)
		fmc = gvs->m_specifiedFMC;			// this is the scenario-level FMC override value
	else if (gvs->m_specifiedFMC_Landscape >= 0.0)
		fmc = gvs->m_specifiedFMC_Landscape;		// this would a project-level (optional inside Canada) FMC override value
	else {
		if (flags & (1ull << CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION)) {
			double x1, y1;
			NumericVariant nv;
			grid::AttributeValue av;
			hr = gvs->grid->GetAttributeData(sts->m_scenario->m_scenario->m_layerThread,
				cpt, sts->m_time, WTimeSpan(0),
				CWFGM_FUELGRID_ATTRIBUTE_X_MID, flags, &nv, &av, nullptr);
			x1 = std::get<double>(nv);
			hr = gvs->grid->GetAttributeData(sts->m_scenario->m_scenario->m_layerThread,
				cpt, sts->m_time, WTimeSpan(0),
				CWFGM_FUELGRID_ATTRIBUTE_Y_MID, flags, &nv, &av, nullptr);
			y1 = std::get<double>(nv);

			bool success = sts->m_scenario->m_coordinateConverter.SourceToLatlon(1, &x1, &y1, nullptr);

			longitude = DEGREE_TO_RADIAN(x1);
			latitude = DEGREE_TO_RADIAN(y1);

				// only maybe BurnP3 would force this option off
		}
		if (flags & (1ull << CWFGM_SCENARIO_OPTION_FMC_TERRAIN)) {
			weak_assert(elev_valid != grid::TerrainValue::NOT_SET);
			double elev;
			if (elev_valid == grid::TerrainValue::DEFAULT) {
				if ((gvs->m_defaultElevation >= 0.0) || (gvs->m_defaultElevation == -99.0))
					elev = gvs->m_defaultElevation;
				else if ((gvs->m_specifiedElev_Landscape >= 0.0) || (gvs->m_specifiedElev_Landscape == -99.0))
					elev = gvs->m_specifiedElev_Landscape;
				else
					elev = c_pt.z;
			} else
				elev = c_pt.z;

			hr = fuel->FMC(latitude, longitude, elev, (std::uint16_t)sts->m_time.GetDayOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST), &overrides, &fmc);
		} else {					// in Prometheus, this case shouldn't happen any more
			weak_assert(false);
			hr = fuel->FMC(latitude, longitude, -99.0, (std::uint16_t)sts->m_time.GetDayOfYear(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST), &overrides, &fmc);
		}
	}
	IWXData wx;
	IFWIData ifwi;
	DFWIData dfwi;
	bool wx_valid;

	hr = gvs->grid->GetWeatherData(sts->m_scenario->m_scenario->m_layerThread,
		    cpt, sts->m_time,
			flags & ((1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMPORAL) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL)
			    | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_PRECIP) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND) | (1ull << (CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND_VECTOR))
				| (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMP_RH)
			    | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI) | (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY)),
		    &wx, &ifwi, &dfwi, &wx_valid, nullptr);

    #ifdef DEBUG_TEST_SLOPE
	wx.WindDirection = 0.0;
	wx.WindSpeed = 0.0;
    #endif

	if (FAILED(hr) || (!wx_valid)) {					// if we couldn't get weather data, for any reason...
		m_ellipse_ros.x = m_ellipse_ros.y = 0.0;
		m_status = FP_FLAG_NOFUEL;
		return;
	}

	if (gvs->self_fire_timestep->m_scenario->m_scenario->m_sc.RH) {
		if (wx.RH < gvs->self_fire_timestep->m_scenario->m_scenario->m_sc.RHThreshold)
			((ScenarioTimeStep<_type> *)gvs->self_fire_timestep)->m_stopConditions.RH = true;	// we can continue to burn
	}

	if (gvs->m_owd != -1.0)
		wx.WindDirection = gvs->m_owd;
	else if ((gvs->target) && (gvs->target_idx != (ULONG)-1) && (gvs->target_sub_idx != (ULONG)-1)) {
		XY_Point to;
		hr = gvs->target->GetTarget(gvs->target_idx, gvs->target_sub_idx, &to);
		if (SUCCEEDED(hr)) {
			sts->m_scenario->toInternal(to);
			XY_Point from(c_pt.x, c_pt.y);
			wx.WindDirection = to.AngleTo(from);			// wx.WindDirection is where the wind is coming from, not where it's going to, here
		}
		else {
			m_ellipse_ros.x = m_ellipse_ros.y = 0.0;
			m_status = FP_FLAG_NOWIND;
			return;
		}
	}
	if (gvs->m_dwd != 0.0)
		wx.WindDirection -= gvs->m_dwd;

	if (wx.Temperature < -50.0)	wx.Temperature = -50.0;
	else if (wx.Temperature > 60.0)	wx.Temperature = 60.0;

	if (wx.RH < 0.0)		wx.RH = 0.0;
	else if (wx.RH > 1.0)		wx.RH = 1.0;

	if (wx.WindSpeed > 200.0)	wx.WindSpeed = 200.0;
	else if (wx.WindSpeed < 0.0)	wx.WindSpeed = 0.0;

	hr = sts->m_scenario->m_scenario->m_fwi->FF(ifwi.FFMC, gvs->day_portion.GetTotalSeconds(), &ff);

	double wsv, rsi, roseq, ros, frss, froseq, fros, brss, broseq, bros, raz;

	if (!(gvs->self_fire->Ignition()->m_ignitionHasPoint))
		flags &= ~(1ull << CWFGM_SCENARIO_OPTION_ACCEL);
	else if (!accel_dtime.GetTotalSeconds())
			accel_dtime = WTimeSpan(0, 0, 0, 1);
	
					// Cordy said, only the point ignition should have acceleration, line and polygon ignitions should NOT do that. Jan.08, 2004
	NumericVariant greenup_on;
	grid::AttributeValue greenup_valid;
	hr = gvs->grid->GetAttributeData(sts->m_scenario->m_scenario->m_layerThread,
	    cpt, sts->m_time, WTimeSpan(0),
	    CWFGM_SCENARIO_OPTION_GREENUP, flags, &greenup_on, &greenup_valid, nullptr);
	if (SUCCEEDED(hr) && (greenup_valid != grid::AttributeValue::NOT_SET)) {
		bool g_on;
		bool b = variantToBoolean(greenup_on, &g_on);
		if (b) {
			if (g_on)
				flags |= (1ull << CWFGM_SCENARIO_OPTION_GREENUP);
			else
				flags &= (~(1ull << CWFGM_SCENARIO_OPTION_GREENUP));
		}
	}

	bool grass;
	if (SUCCEEDED(fuel->IsGrassFuelType(&grass)) && (grass)) {
		NumericVariant standing_on;
		grid::AttributeValue standing_valid;
		hr = gvs->grid->GetAttributeData(sts->m_scenario->m_scenario->m_layerThread,
			cpt, sts->m_time, WTimeSpan(0),
			CWFGM_SCENARIO_OPTION_GRASSPHENOLOGY, flags, &standing_on, &standing_valid, nullptr);
		if (SUCCEEDED(hr) && (standing_valid != grid::AttributeValue::NOT_SET)) {
			bool g_on;
			bool b = variantToBoolean(standing_on, &g_on);
			if (b) {
				if (g_on)
					flags |= (1ull << CWFGM_SCENARIO_OPTION_GRASSPHENOLOGY);
				else
					flags &= (~(1ull << CWFGM_SCENARIO_OPTION_GRASSPHENOLOGY));
			}
		}
	}

	double lb;

	double windSpeed = sts->m_scenario->m_scenario->m_impl->m_go.ApplyGusting(gvs->self_fire, sts->m_time, wx.WindSpeed, wx.WindGust);

	hr = fuel->CalculateROSValues(aspect, azimuth, windSpeed, wx.WindDirection + CONSTANTS_NAMESPACE::Pi<double>(), dfwi.dBUI, fmc, ifwi.FFMC, ff,
		accel_dtime, gvs->day_portion, (std::int16_t)(flags & 0xffff), &overrides, sts->m_scenario->m_scenario,
		&rsi, &roseq, &ros, &frss, &froseq, &fros, &brss, &broseq, &bros, &lb, &wsv, &raz);

	if (roseq > sts->m_scenario->m_scenario->m_minimumROS) {
		if (roseq < 1e-5)
			m_fbp_ros_ratio = 1.0;
		else	
			m_fbp_ros_ratio = ros / roseq;

		m_fbp_rsi = rsi;
		m_fbp_roseq = roseq;
		m_fbp_ros = ros;
		m_fbp_bros = bros;
		m_fbp_fros = fros;
		m_fbp_raz = CARTESIAN_TO_COMPASS_RADIAN(raz);
	
		if (flags & (1ull << CWFGM_SCENARIO_OPTION_USE_2DGROWTH)) {
			grow2D(p_pt, s_pt);
		} else { /* want 3-d growth */
			grow3D(sts, c_pt, p_pt, s_pt, aspect, azimuth);
		}

		m_vector_ros = m_ellipse_ros.Length();

		double cfb, cfc, sfc, tfc, fi, ta1, ta2;

		fuel->CalculateFCValues(ifwi.FFMC, dfwi.dBUI, fmc, m_fbp_rsi, m_fbp_ros, (std::int16_t)(flags & 0xffff), &overrides, &cfb, &cfc, &ta1, &ta2, &sfc, &tfc, &fi);
		m_fbp_cfb = cfb;
		m_fbp_fi = fi;

		fuel->CalculateFCValues(ifwi.FFMC, dfwi.dBUI, fmc, m_vector_ros, m_vector_ros, (std::int16_t)(flags & 0xffff), &overrides, &cfb, &cfc, &ta1, &ta2, &sfc, &tfc, &fi);
		m_vector_cfb = cfb;
		m_vector_cfc = cfc;
		m_vector_sfc = sfc;
		m_vector_tfc = tfc;
		m_vector_fi = fi;
		m_flameLength = flameLength(fuel, cfb, fi, &overrides);

	} else {

		m_ellipse_ros.x = m_ellipse_ros.y = 0.0;
		m_fbp_ros_ratio = 1.0;
	}

	if (!sts->m_scenario->CanBurn(sts->m_time, gvs->centroid, cpt, wx.RH, windSpeed, ifwi.FWI, ifwi.ISI)) {
		m_ellipse_ros.x = m_ellipse_ros.y = 0.0;
		m_fbp_ros_ratio = 1.0;
	}
	// collect the values of interest for us to store for this point
}

template<class _type>
std::uint32_t AFX_CDECL growVoxelInit(APTR parameter) {
	growVoxelIterator<_type> *gvs = (growVoxelIterator<_type>*)parameter;

	growVoxelParms<_type> cgvs(gvs->gvp);

	while (1) {
		std::uint32_t cnt = 0, l1 = 0, l2 = 0;
		gvs->lock_ll.Lock();
		ScenarioFire<_type> *sf_loop = gvs->sf;
		FireFront<_type> *ff_loop = gvs->ff;
		FirePoint<_type> *end_fp, *fp_loop = gvs->fp;

		for (; gvs->sf->LN_Succ(); gvs->sf = gvs->sf->LN_Succ(), l1 = 1) {
			if (l1)
				gvs->ff = gvs->sf->LH_Head();
			for (; gvs->ff->LN_Succ(); gvs->ff = gvs->ff->LN_Succ(), l2 = 1) {
				if (l2)
					gvs->fp = gvs->ff->LH_Head();
				for (; gvs->fp->LN_Succ(); gvs->fp = gvs->fp->LN_Succ()) {
					if (cnt >= gvs->gvp.queue_up) {
						goto DONE_SCAN;
					}
					if (gvs->fp->m_status == FP_FLAG_NORMAL)
						cnt++;					// we find QUEUE_UP # of points to grow
				}
			}
		}


DONE_SCAN:
		end_fp = gvs->fp;

		gvs->lock_ll.Unlock();					// let other threads have access to this struct

		if (!cnt) {						// if we're at the end of the list
			return 1;					// return 'cause we're done this job
		}

		l1 = l2 = 0;
		for (; sf_loop->LN_Succ(); sf_loop = sf_loop->LN_Succ(), l1 = 1) {
		  if (l1)
			  ff_loop = sf_loop->LH_Head();
		  cgvs.self_fire = sf_loop;
		  cgvs.accel_dtime = cgvs.self_fire_timestep->m_time - sf_loop->Ignition()->m_ignitionTime;
		  for (; ff_loop->LN_Succ(); ff_loop = ff_loop->LN_Succ(), l2 = 1) {
		    if (l2)
			    fp_loop = ff_loop->LH_Head();
		    cgvs.self = ff_loop;
		    for (; fp_loop->LN_Succ(); fp_loop = fp_loop->LN_Succ()) {
					if (fp_loop == end_fp)
						goto DONE_GROW;
					if (fp_loop->m_status == FP_FLAG_NORMAL) {
						bool valid;
						ICWFGM_Fuel *fuel = sf_loop->TimeStep()->m_scenario->GetFuel(sf_loop->TimeStep()->m_time, *fp_loop, valid);
						bool result;
						if ((valid) && (fuel) && (SUCCEEDED(fuel->IsNonFuel(&result))) && (!result)) {

							fp_loop->Grow(&cgvs, fuel);
						} else {
							fp_loop->m_ellipse_ros.x = fp_loop->m_ellipse_ros.y = 0.0;
							fp_loop->m_fbp_ros_ratio = 1.0;
							fp_loop->m_status = FP_FLAG_NOFUEL;
						}
					}
		    }
		  }
		}
DONE_GROW:
		;
	}
}


template<class _type>
void FireFront<_type>::GrowPoints() {
	growVoxelParms<_type> gvs;
	Scenario<_type> *s = Fire()->TimeStep()->m_scenario;
	gvs.self_fire_timestep = Fire()->TimeStep();
	(const_cast<ScenarioTimeStep<_type> *>(Fire()->TimeStep()))->Centroid(&gvs.centroid);
	gvs.self_fire = (ScenarioFire<_type>*)Fire();
	gvs.self = this;
	gvs.grid = s->m_scenario->m_gridEngine.get();
	gvs.latitude = s->m_scenario->m_timeManager->m_worldLocation.m_latitude();
	gvs.longitude = s->m_scenario->m_timeManager->m_worldLocation.m_longitude();
	gvs.accel_dtime = Fire()->TimeStep()->m_time - Fire()->Ignition()->m_ignitionTime;
	gvs.day_portion = Fire()->TimeStep()->m_time.GetTimeOfDay(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	if (s->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_SPECIFIED_FMC_ACTIVE)))
		gvs.m_specifiedFMC = s->m_scenario->m_specifiedFMC;
	else
		gvs.m_specifiedFMC = -1.0;
	gvs.m_specifiedFMC_Landscape = s->m_specifiedFMC_Landscape;
	gvs.m_defaultElevation = s->m_scenario->m_defaultElevation;
	gvs.m_specifiedElev_Landscape = s->m_specifiedElev_Landscape;
	if (s->m_scenario->m_dwd != 0.0)
		gvs.m_dwd = DEGREE_TO_RADIAN(s->m_scenario->m_dwd);
	else
		gvs.m_dwd = 0.0;

	if (s->m_scenario->m_owd != -1.0)
		gvs.m_owd = DEGREE_TO_RADIAN(COMPASS_TO_CARTESIAN_DEGREE(s->m_scenario->m_owd));
	else
		gvs.m_owd = -1.0;

	gvs.target = s->m_scenario->m_windTarget.get();
	gvs.target_idx = s->m_scenario->m_windTargetIndex;
	gvs.target_sub_idx = s->m_scenario->m_windTargetSubIndex;

	FirePoint<_type> *fp = LH_Head();
	while (fp->LN_Succ()) {
		if (fp->m_status == FP_FLAG_NORMAL) {
			bool valid;
			ICWFGM_Fuel *fuel = gvs.self_fire_timestep->m_scenario->GetFuel(gvs.self_fire_timestep->m_time, *fp, valid);
			bool result;
			if ((valid) && (fuel) && (SUCCEEDED(fuel->IsNonFuel(&result))) && (!result)) {
				fp->Grow(&gvs, fuel);
			} else {
				fp->m_ellipse_ros.x = fp->m_ellipse_ros.y = 0.0;
				fp->m_fbp_ros_ratio = 1.0;
				fp->m_status = FP_FLAG_NOFUEL;
				}
		}
		fp = fp->LN_Succ();
	}
}


template<class _type>
void ScenarioFire<_type>::StatsFires() {						// calculates new FBP speeds, directions for each active fire vertex
	FireFront<_type> *ff = LH_Head();
	while (ff->LN_Succ()) {
		ff->GrowPoints();
		ff = ff->LN_Succ();
	}
}


template<class _type>
void ScenarioTimeStep<_type>::StatsFires() {
	std::uint32_t total_num_points = 0, total_num_polys = 0;

	ScenarioFire<_type> *sf = m_fires.LH_Head();
	while (sf->LN_Succ()) {
		total_num_points += sf->NumPoints();
		total_num_polys += sf->NumPolys();
		sf = sf->LN_Succ();
	}

	std::uint32_t queue_up;
	if (m_scenario->m_scenario->m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL))
		queue_up = QUEUE_UP;
	else
		queue_up = QUEUE_UP >> 1;

	if ((total_num_points > queue_up) && (m_scenario->m_pool)) {
		growVoxelIterator<_type> gvs;
		gvs.gvp.self_fire_timestep = this;
		Centroid(&gvs.gvp.centroid);
		gvs.gvp.grid = m_scenario->m_scenario->m_gridEngine.get();
		gvs.gvp.latitude = m_scenario->m_scenario->m_timeManager->m_worldLocation.m_latitude();
		gvs.gvp.longitude = m_scenario->m_scenario->m_timeManager->m_worldLocation.m_longitude();
		gvs.gvp.day_portion = m_time.GetTimeOfDay(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
		if (m_scenario->m_scenario->m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_SPECIFIED_FMC_ACTIVE)))
		gvs.gvp.m_specifiedFMC = m_scenario->m_scenario->m_specifiedFMC;
		else
			gvs.gvp.m_specifiedFMC = -1.0;
		gvs.gvp.m_specifiedFMC_Landscape = m_scenario->m_specifiedFMC_Landscape;
		gvs.gvp.m_defaultElevation = m_scenario->m_scenario->m_defaultElevation;
		gvs.gvp.m_specifiedElev_Landscape = m_scenario->m_specifiedElev_Landscape;

		if (m_scenario->m_scenario->m_dwd != 0.0)
			gvs.gvp.m_dwd = DEGREE_TO_RADIAN(m_scenario->m_scenario->m_dwd);
		else
			gvs.gvp.m_dwd = 0.0;

		if (m_scenario->m_scenario->m_owd != -1.0)
			gvs.gvp.m_owd = DEGREE_TO_RADIAN(COMPASS_TO_CARTESIAN_DEGREE(m_scenario->m_scenario->m_owd));
		else
			gvs.gvp.m_owd = -1.0;

		gvs.gvp.target = m_scenario->m_scenario->m_windTarget.get();
		gvs.gvp.target_idx = m_scenario->m_scenario->m_windTargetIndex;
		gvs.gvp.target_sub_idx = m_scenario->m_scenario->m_windTargetSubIndex;

		gvs.gvp.queue_up = queue_up;

		for (gvs.sf = m_fires.LH_Head(); gvs.sf->LN_Succ(); gvs.sf = gvs.sf->LN_Succ())
			for (gvs.ff = gvs.sf->LH_Head(); gvs.ff->LN_Succ(); gvs.ff = gvs.ff->LN_Succ())
				for (gvs.fp = gvs.ff->LH_Head(); gvs.fp->LN_Succ(); gvs.fp = gvs.fp->LN_Succ()) {
					//move to current thread for testing
					weak_assert(gvs.sf);
					m_scenario->m_pool->SetJobFunction(growVoxelInit<_type>, &gvs);
					m_scenario->m_pool->StartJob();
					m_scenario->m_pool->BlockOnJob();
					goto DONE;
				}
DONE:
		;
	} else {
		ScenarioFire<_type> *ff = m_fires.LH_Head();
		while (ff->LN_Succ()) {
			ff->StatsFires();
			ff = ff->LN_Succ();
		}
	}
}


template<class _type>
bool ScenarioTimeStep<_type>::CheckAssets(bool& make_displayable) {
	bool exit = false;
	std::uint32_t count = m_scenario->m_scenario->m_impl->m_assetList.GetCount(),
		fullCount = 0;
	if (count > 0) {
		std::uint32_t globalCount = 0;
		AssetNode<_type>* an = m_scenario->m_scenario->m_impl->m_assetList.LH_Head();
		while (an->LN_Succ()) {
			std::uint32_t anCount = 0;
			AssetGeometryNode<_type>* agn = an->m_geometry.LH_Head();
			while (agn->LN_Succ()) {
				if (m_time == m_scenario->m_scenario->m_startTime) {
					if (agn->m_arrived) {
						anCount++;
						make_displayable = true;
					}
				}

				if (!agn->m_arrived) {
					if ((agn->m_geometry.IsMultiPoint())) {
						auto pt = agn->m_geometry.LH_Head();
						bool inside = false;
						while (pt->LN_Succ()) {
							if (PointInArea(*pt)) {
								inside = true;
								break;
							}
							pt = pt->LN_Succ();
						}
						if (inside) {
							agn->m_arrivalTime = m_time;
							agn->m_arrived = true;
							agn->m_closestFirePoint = GetNearestPoint(*pt, true, &agn->m_closestFireFront, true);
							agn->m_closestPoint.copyValuesFrom(*agn->m_closestFirePoint);
							agn->fixClosestPoint();
							m_assetCount++;
							anCount++;
							make_displayable = true;
						}
					}
					else {
						ScenarioFire<_type>* sf = this->m_fires.LH_Head();
						bool intersects = false;

						while (sf->LN_Succ()) {
							if (sf->FastCollisionTest(agn->m_geometry, 0.0)) {
								FireFront<_type>* ff = sf->LH_Head();
								while (ff->LN_Succ()) {
									if (ff->Intersects(agn->m_geometry)) {
										intersects = true;
										break;
									}
									ff = ff->LN_Succ();
								}
							}
							if (intersects)
								break;
							sf = sf->LN_Succ();
						}
						if (intersects) {
							agn->m_arrivalTime = m_time;
							agn->m_arrived = true;
							m_assetCount++;
							anCount++;
							make_displayable = true;
						}
					}
				}
				else
					anCount++;
				agn = agn->LN_Succ();
			}

			if (an->m_operation == (std::uint32_t)-1) {
				if (anCount == an->m_geometry.GetCount())
					exit = true;
			} else if (an->m_operation > 0) {
				if (anCount >= an->m_operation)
					exit = true;
			}

			globalCount += anCount;
			fullCount += an->m_geometry.GetCount();
			an = an->LN_Succ();
		}

		if (m_scenario->m_scenario->m_globalAssetOperation == (std::uint32_t)-1) {
			if (globalCount == fullCount)
				exit = true;
		} else if (m_scenario->m_scenario->m_globalAssetOperation > 0) {
			if (globalCount >= m_scenario->m_scenario->m_globalAssetOperation)
				exit = true;
		}
	}

	return exit;
}

#include "InstantiateClasses.cpp"
