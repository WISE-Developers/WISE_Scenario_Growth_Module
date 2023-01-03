/**
 * WISE_Scenario_Growth_Module: firepoint.h
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

#ifndef __FIREPOINT_H
#define __FIREPOINT_H

#include "poly.h"
#include "vectors.h"
#include "FireEngine.h"


template<class _type>
struct growVoxelParms;
template<class _type>
class ScenarioTimeStep;


template<class _type>
class FirePoint : public XY_PolyLLNode<_type> {
	using XYPointType = XY_PointTempl<_type>;
	using XYZPointType = XYZ_PointTempl<_type>;
	using XYPolyLLType = XY_PolyLL_BaseTempl<_type>;
	using XYVectorType = XY_VectorTempl<_type>;

public:
	using XY_PolyLLNode<_type>::x;
	using XY_PolyLLNode<_type>::y;

public:
	DECLARE_OBJECT_CACHE_MT(FirePoint<_type>, FirePoint)

	FirePoint();
	FirePoint(const XYPointType &pt);
	FirePoint(const XY_PolyLLNode<_type> &pt);
	FirePoint(const FirePoint<_type> &fp);						// for all constructors, all STATS's are automatically cleared when creating a new FirePoint
	~FirePoint();

	__INLINE FirePoint<_type> *LN_Succ() const				{ return (FirePoint<_type>*)XY_PolyLLNode<_type>::LN_Succ(); };
	__INLINE FirePoint<_type> *LN_Pred() const				{ return (FirePoint<_type>*)XY_PolyLLNode<_type>::LN_Pred(); };
	__INLINE FirePoint<_type> *LN_SuccWrap() const			{ return (FirePoint<_type>*)XY_PolyLLNode<_type>::LN_SuccWrap(); };
	__INLINE FirePoint<_type> *LN_PredWrap() const			{ return (FirePoint<_type>*)XY_PolyLLNode<_type>::LN_PredWrap(); };

	#define FP_FLAG_NORMAL			0			// the point is happily growing along
	#define FP_FLAG_NO_ROS			1			// the point stopped because the fuel (which was valid) couldn't support any ROS (like D2 with greenup and low BUI)
	#define FP_FLAG_NOFUEL			2			// the point is stopped 'cause it ran into fuel that won't burn, or no fuel
	#define FP_FLAG_VECTOR			3			// the point is stopped 'cause it ran into a vector firebreak
	#define FP_FLAG_FIRE			4			// the point is stopped 'cause it ran into another fire
	#define FP_FLAG_NOWIND			5			// this is specific to a point being stopped because we were supposed to aim the point at a target, but
												// we failed to get the target to aim at

	__INLINE FirePoint &operator=(const XYPointType &pt)	{ if (&pt != this) { XYPointType::x = pt.x; XYPointType::y = pt.y; } return *this; };

	__INLINE bool CanMove() const							{ if (m_status) return false; if ((m_ellipse_ros.x == 0.0) && (m_ellipse_ros.y)) return false; return true; }

	FirePoint<_type>	*m_prevPoint, *m_succPoint;
	XYVectorType		m_ellipse_ros;
	_type				m_fbp_raz;							// IN COMPASS
	std::uint32_t	m_status : 4,
						m_successful_breach : 1;

	double			m_fbp_rsi, m_fbp_roseq, m_fbp_ros, m_fbp_bros, m_fbp_fros, m_vector_ros;
															// the above are "pure" values from the FBP (FuelCOM) engine, unmodified, for this specific point
	double			m_vector_cfb, m_vector_cfc, m_vector_sfc, m_vector_tfc, m_vector_fi;

	double			m_fbp_fi, m_fbp_cfb, m_fbp_ros_ratio, m_flameLength;
															// the m_vector_* variables are for the vector of growth (as determined by the ellipse model) so
															// are again specific to this point, but also determined by the points neighbours (locations of)
	static double flameLength(ICWFGM_Fuel *fuel, double cfb, double fi, const CCWFGM_FuelOverrides *overrides);

	HRESULT RetrieveStat(const std::uint16_t stat, double &s) const;
	HRESULT RetrieveAttribute(const std::uint16_t stat, const uint32_t units, GDALVariant& a) const;

	void Grow(const growVoxelParms<_type> *gvs, ICWFGM_Fuel *fuel);

private:

    #ifdef TEST_JONATHAN
	void grow2D(const XYZPointType &p_pt, const XYZPointType &s_pt, const std::uint32_t num_points);
    #else
	void grow2D(const XYZPointType &p_pt, const XYZPointType &s_pt);
    #endif

	void grow3D(const ScenarioTimeStep<_type> *timeStep, const XYZPointType &c_pt, const XYZPointType &p_pt, const XYZPointType &s_pt, _type aspect, const _type azimuth);

public:
	void copyValuesFrom(const FirePoint& toCopy);		// like a copy operator but don't want to override that operator over possible other issues
};

#endif
