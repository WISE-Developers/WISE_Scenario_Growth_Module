/**
 * WISE_Scenario_Growth_Module: firepoint.cpp
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
#include "firepoint.h"
#include "FireEngine_ext.h"
#include "FuelCom_ext.h"
#include "results.h"
#include "propsysreplacement.h"


IMPLEMENT_OBJECT_CACHE_MT_TEMPLATE(FirePoint, FirePoint, 1024*1024*256/sizeof(FirePoint<fireengine_float_type>), true, 16, fireengine_float_type)


template<class _type>
FirePoint<_type>::FirePoint() {
	m_prevPoint = nullptr;
	m_succPoint = nullptr;
}


template<class _type>
FirePoint<_type>::FirePoint(const FirePoint<_type> &fp) {
	m_prevPoint = (FirePoint<_type>*)&fp;
	m_succPoint = nullptr;
	x = fp.x;
	y = fp.y;
	m_status = fp.m_status;
	if (m_status)
		m_fbp_ros_ratio = 1.0;
}


template<class _type>
FirePoint<_type>::FirePoint(const XYPointType &pt) {
	m_prevPoint = nullptr;
	m_succPoint = nullptr;
	x = pt.x;
	y = pt.y;
}


template<class _type>
FirePoint<_type>::FirePoint(const XY_PolyLLNode<_type> &pt) {
	m_prevPoint = nullptr;
	m_succPoint = nullptr;
	x = pt.x;
	y = pt.y;
}


template<class _type>
FirePoint<_type>::~FirePoint() {
}


template<class _type>
double FirePoint<_type>::flameLength(ICWFGM_Fuel *fuel, double cfb, double fi, const CCWFGM_FuelOverrides *overrides) {
	if (!fuel)
		return 0.0;

	double fl;
	double height = 0.0;
	PolymorphicAttribute v;
	if (SUCCEEDED(fuel->GetAttribute(FUELCOM_ATTRIBUTE_TREE_HEIGHT, &v)))
	    VariantToDouble_(v, &height);
	fuel->FlameLength(height, cfb, fi, overrides, &fl);
	return fl;
}


template<class _type>
HRESULT FirePoint<_type>::RetrieveStat(const std::uint16_t stat, double &s) const {
	if (!m_status) {
		switch (stat) {
			case CWFGM_FIRE_STAT_FBP_RSI:		s = m_fbp_rsi; break;
			case CWFGM_FIRE_STAT_FBP_ROSEQ:		s = m_fbp_roseq; break;
			case CWFGM_FIRE_STAT_FBP_ROS:		s = m_fbp_ros; break;
			case CWFGM_FIRE_STAT_FBP_BROS:		s = m_fbp_bros; break;
			case CWFGM_FIRE_STAT_FBP_FROS:		s = m_fbp_fros; break;
			case CWFGM_FIRE_STAT_RAZ:
							
    #ifdef DEBUG
												weak_assert(m_fbp_raz >= 0.0);
												weak_assert(m_fbp_raz <= CONSTANTS_NAMESPACE::TwoPi<double>());
    #endif

												s = COMPASS_TO_CARTESIAN_RADIAN(m_fbp_raz); break;

			case CWFGM_FIRE_STAT_ROS:			s = m_vector_ros; break;
			case CWFGM_FIRE_STAT_CFB:			s = m_vector_cfb; break;
			case CWFGM_FIRE_STAT_HCFB:			s = m_fbp_cfb; break;
			case CWFGM_FIRE_STAT_CFC:			s = m_vector_cfc; break;
			case CWFGM_FIRE_STAT_SFC:			s = m_vector_sfc; break;
			case CWFGM_FIRE_STAT_TFC:			s = m_vector_tfc; break;
			case CWFGM_FIRE_STAT_FI:			s = m_vector_fi; break;
			case CWFGM_FIRE_STAT_HFI:			s = m_fbp_fi; break;
			case CWFGM_FIRE_STAT_FLAMELENGTH:	s = m_flameLength; break;

			case CWFGM_FIRE_STAT_ACTIVE:		s = 1.0; break;

			default:							return ERROR_FIRE_STAT_UNKNOWN;
		}
	} else {
		switch (stat) {
			case CWFGM_FIRE_STAT_ACTIVE:
			case CWFGM_FIRE_STAT_FBP_RSI:
			case CWFGM_FIRE_STAT_FBP_ROSEQ:
			case CWFGM_FIRE_STAT_FBP_ROS:
			case CWFGM_FIRE_STAT_FBP_BROS:
			case CWFGM_FIRE_STAT_FBP_FROS:
			case CWFGM_FIRE_STAT_RAZ:

			case CWFGM_FIRE_STAT_ROS:
			case CWFGM_FIRE_STAT_CFB:
			case CWFGM_FIRE_STAT_HCFB:
			case CWFGM_FIRE_STAT_CFC:
			case CWFGM_FIRE_STAT_SFC:
			case CWFGM_FIRE_STAT_TFC:
			case CWFGM_FIRE_STAT_FI:
			case CWFGM_FIRE_STAT_HFI:
			case CWFGM_FIRE_STAT_FLAMELENGTH:	s = 0.0; break;
			default:							return ERROR_FIRE_STAT_UNKNOWN;
		}
	}
	return S_OK;
}


template<class _type>
HRESULT FirePoint<_type>::RetrieveAttribute(const std::uint16_t stat, const std::uint32_t units, GDALVariant& a) const {
	double s;
	HRESULT hr = RetrieveStat(stat, s);
	if (SUCCEEDED(hr)) {
		if (units) {
			switch (stat) {
			case CWFGM_FIRE_STAT_FBP_RSI:
			case CWFGM_FIRE_STAT_FBP_ROSEQ:
			case CWFGM_FIRE_STAT_FBP_ROS:
			case CWFGM_FIRE_STAT_FBP_BROS:
			case CWFGM_FIRE_STAT_FBP_FROS:
			case CWFGM_FIRE_STAT_ROS:			s = UnitConvert::convertUnit(s, units, STORAGE_FORMAT_M | STORAGE_FORMAT_MINUTE);
												break;
			case CWFGM_FIRE_STAT_FI:
			case CWFGM_FIRE_STAT_HFI:			s = UnitConvert::convertUnit(s, units, ((UnitConvert::STORAGE_UNIT)STORAGE_FORMAT_KILOWATT_SECOND << 0x20) | STORAGE_FORMAT_M);
												break;
			case CWFGM_FIRE_STAT_TFC:
			case CWFGM_FIRE_STAT_SFC:
			case CWFGM_FIRE_STAT_CFC:
												s = UnitConvert::convertUnit(s, units, ((UnitConvert::STORAGE_UNIT)STORAGE_FORMAT_KG << 0x20) | STORAGE_FORMAT_M2);
												break;
			case CWFGM_FIRE_STAT_FLAMELENGTH:	s = UnitConvert::convertUnit(s, units, STORAGE_FORMAT_M);
												break;
			case CWFGM_FIRE_STAT_ACTIVE:
			case CWFGM_FIRE_STAT_CFB:
			case CWFGM_FIRE_STAT_HCFB:
			case CWFGM_FIRE_STAT_RAZ:
			default:							break;
			}
		}
		a = s;
	}
	return hr;
}


template<class _type>
void FirePoint<_type>::copyValuesFrom(const FirePoint& toCopy) {
	m_prevPoint = m_succPoint = nullptr;
	x = toCopy.x;
	y = toCopy.y;
	m_ellipse_ros = toCopy.m_ellipse_ros;
	m_fbp_raz = toCopy.m_fbp_raz;
	m_status = toCopy.m_status;
	m_fbp_rsi = toCopy.m_fbp_rsi;
	m_fbp_roseq = toCopy.m_fbp_roseq;
	m_fbp_ros = toCopy.m_fbp_ros;
	m_fbp_bros = toCopy.m_fbp_bros;
	m_fbp_fros = toCopy.m_fbp_fros;
	m_vector_ros = toCopy.m_vector_ros;
	m_vector_cfb = toCopy.m_vector_cfb;
	m_vector_cfc = toCopy.m_vector_cfc;
	m_vector_sfc = toCopy.m_vector_sfc;
	m_vector_tfc = toCopy.m_vector_tfc;
	m_vector_fi = toCopy.m_vector_fi;
	m_fbp_fi = toCopy.m_fbp_fi;
	m_fbp_cfb = toCopy.m_fbp_cfb;
	m_fbp_ros_ratio = toCopy.m_fbp_ros_ratio;
	m_flameLength = toCopy.m_flameLength;
}


template class FirePoint<fireengine_float_type>;
