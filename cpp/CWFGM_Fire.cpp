/**
 * WISE_Scenario_Growth_Module: CWFGM_Fire.cpp
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

#include "intel_check.h"
#include "propsysreplacement.h"
#include "FireEngine.h"
#include "CWFGM_Fire.h"
#include "GridCom_ext.h"
#include "results.h"
#include "poly.h"
#include "CoordinateConverter.h"

using namespace HSS_Time;


IMPLEMENT_OBJECT_CACHE_MT_NO_TEMPLATE(Ignition, Ignition, 128 * 1024 / sizeof(Ignition), false, 16)


#ifndef DOXYGEN_IGNORE_CODE

CCWFGM_Ignition::CCWFGM_Ignition() : m_timeManager(nullptr), m_startTime((std::uint64_t)0, m_timeManager) {
	m_bRequiresSave = false;
	m_resolution = -1.0;
}


CCWFGM_Ignition::CCWFGM_Ignition(const CCWFGM_Ignition &toCopy) : m_timeManager(toCopy.m_timeManager), m_startTime((std::uint64_t)0, m_timeManager) {
	CRWThreadSemaphoreEngage engage(*(CRWThreadSemaphore *)&toCopy.m_lock, SEM_FALSE);

	m_bRequiresSave = false;

	m_gisURL = toCopy.m_gisURL;
	m_gisLayer = toCopy.m_gisLayer;
	m_gisUID = toCopy.m_gisUID;
	m_gisPWD = toCopy.m_gisPWD;

	m_startTime.SetTime(toCopy.m_startTime);

	m_resolution = toCopy.m_resolution;

	Ignition *i = toCopy.m_ignitionList.LH_Head();
	while (i->LN_Succ()) {
		Ignition *ii = new Ignition(*i);
		m_ignitionList.AddTail(ii);
		i = i->LN_Succ();
	}
}


Ignition::Ignition() {
	m_ignitionPolyType = CWFGM_FIRE_IGNITION_UNDEFINED;
	m_ignition = nullptr;
}


Ignition::Ignition(const Ignition &toCopy) {
	m_ignitionPolyType = toCopy.m_ignitionPolyType;
	if (toCopy.m_ignition)
		m_ignition = new XY_Poly(*toCopy.m_ignition);
	else
		m_ignition = nullptr;
}


Ignition::~Ignition() {
	if (m_ignition)
		delete m_ignition;
}


CCWFGM_Ignition::~CCWFGM_Ignition() {
	Ignition *ig;
	while (ig = m_ignitionList.RemHead())
		delete ig;
}


HRESULT CCWFGM_Ignition::put_CommonData(ICWFGM_CommonData* pVal) {
	if (!pVal)
		return E_POINTER;
	m_timeManager = pVal->m_timeManager;
	m_startTime.SetTimeManager(m_timeManager);
	return S_OK;
}


HRESULT CCWFGM_Ignition::fixResolution() {
	HRESULT hr;
	double gridResolution, temp;
	PolymorphicAttribute var;

	boost::intrusive_ptr<ICWFGM_GridEngine> gridEngine;
	if (!(gridEngine = m_gridEngine))					{ weak_assert(false); return ERROR_GRID_UNINITIALIZED; }

	if (!m_timeManager) {
		weak_assert(0);
		ICWFGM_CommonData* data;
		if (FAILED(hr = gridEngine->GetCommonData(nullptr, &data)) || (!data)) return hr;
		m_timeManager = data->m_timeManager;
		m_startTime.SetTimeManager(m_timeManager);
	}
	if (FAILED(hr = gridEngine->GetAttribute(nullptr, CWFGM_GRID_ATTRIBUTE_PLOTRESOLUTION, &var)))  return hr; VariantToDouble_(var, &gridResolution);

	m_resolution = gridResolution;
	return S_OK;
}

#endif

HRESULT CCWFGM_Ignition::Clone(boost::intrusive_ptr<ICWFGM_CommonBase> *newObject) const {
	if (!newObject)							return E_POINTER;

	CRWThreadSemaphoreEngage engage(*(CRWThreadSemaphore *)&m_lock, SEM_FALSE);

	try {
		CCWFGM_Ignition *f = new CCWFGM_Ignition(*this);
		*newObject = f;
		return S_OK;
	}
	catch (std::exception &e) {
	}
	return E_FAIL;
}


HRESULT CCWFGM_Ignition::get_GridEngine(boost::intrusive_ptr<ICWFGM_GridEngine> *pVal) {
	if (!pVal)								return E_POINTER;
	*pVal = m_gridEngine;
	if (!m_gridEngine)							{ weak_assert(false); return ERROR_IGNITION_UNINITIALIZED; }
	return S_OK;
}


HRESULT CCWFGM_Ignition::put_GridEngine(ICWFGM_GridEngine *newVal) {
	if (newVal) {
		boost::intrusive_ptr<ICWFGM_GridEngine> pGridEngine;
		pGridEngine = dynamic_cast<ICWFGM_GridEngine*>(const_cast<ICWFGM_GridEngine*>(newVal));
		if (pGridEngine.get()) {
			m_gridEngine = pGridEngine;
			fixResolution();
			return S_OK;
		}
		return E_FAIL;
	}

	m_gridEngine = newVal;
	return S_OK;
}


HRESULT CCWFGM_Ignition::Valid(const HSS_Time::WTime &start_time, const HSS_Time::WTimeSpan &duration) {
	if (!m_gridEngine) { weak_assert(false); return ERROR_IGNITION_UNINITIALIZED; }
	return S_OK;
}


HRESULT CCWFGM_Ignition::MT_Lock(bool exclusive, std::uint16_t obtain) {
	if (obtain == (std::uint16_t)-1) {
		std::int64_t state = m_lock.CurrentState();
		if (!state)				return SUCCESS_STATE_OBJECT_UNLOCKED;
		if (state < 0)			return SUCCESS_STATE_OBJECT_LOCKED_WRITE;
		if (state >= 1000000LL)	return SUCCESS_STATE_OBJECT_LOCKED_SCENARIO;
		return						   SUCCESS_STATE_OBJECT_LOCKED_READ;
	} else if (obtain) {
		if (exclusive)	m_lock.Lock_Write();
		else			m_lock.Lock_Read(1000000LL);
	} else {
		if (exclusive)	m_lock.Unlock();
		else			m_lock.Unlock(1000000LL);
	}
	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionTime(HSS_Time::WTime *pVal) {
	if (!pVal)							return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, SEM_FALSE);

	pVal->SetTime(m_startTime);
	return S_OK;
}


HRESULT CCWFGM_Ignition::SetIgnitionTime(const HSS_Time::WTime &newVal) {
	SEM_BOOL engaged;
	CRWThreadSemaphoreEngage engage(m_lock, SEM_TRUE, &engaged, 1000000LL);
	if (!engaged)								return ERROR_SCENARIO_SIMULATION_RUNNING;

	HSS_Time::WTime val(newVal);
	val.PurgeToSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);

	m_startTime.SetTime(val);

#ifdef DEBUG
	std::string stime = m_startTime.ToString(WTIME_FORMAT_STRING_ISO8601);
#endif

	m_bRequiresSave = true;
	return S_OK;
}


HRESULT CCWFGM_Ignition::AddIgnition(std::uint16_t ignition_type, const XY_PolyConst &xy_pairs, std::uint32_t *index) {
	SEM_BOOL engaged;
	CRWThreadSemaphoreEngage engage(m_lock, SEM_TRUE, &engaged, 1000000LL);
	if (!engaged)									return ERROR_SCENARIO_SIMULATION_RUNNING;

	if ((ignition_type != CWFGM_FIRE_IGNITION_POINT) && (xy_pairs.NumPoints() == 1)) {
		return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
	}

	if ((ignition_type == CWFGM_FIRE_IGNITION_LINE) && (xy_pairs.NumPoints() < 2))		return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
	if ((ignition_type == CWFGM_FIRE_IGNITION_POLYGON_IN) && (xy_pairs.NumPoints() < 2))	return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
	if ((ignition_type == CWFGM_FIRE_IGNITION_POLYGON_OUT) && (xy_pairs.NumPoints() < 2))	return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;

	if (xy_pairs.NumPoints() == 0)								return E_INVALIDARG;

	std::uint16_t flags;
	if ((ignition_type == CWFGM_FIRE_IGNITION_POLYGON_IN) || (ignition_type == CWFGM_FIRE_IGNITION_POLYGON_OUT))		flags = XY_Poly::PolygonType::POLYGON;
	else if (ignition_type == CWFGM_FIRE_IGNITION_LINE)																	flags = XY_Poly::PolygonType::POLYLINE;
	else if (ignition_type == CWFGM_FIRE_IGNITION_POINT)																flags = XY_Poly::PolygonType::MULTIPOINT;
	else {
		weak_assert(false);
		return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
	}
	XY_Poly *pts = nullptr;
	Ignition *m_ignition = nullptr;
	try {
		m_ignition = new Ignition();

		std::uint32_t i;
		pts = new XY_Poly(xy_pairs);

		pts->CleanPoly(0.0, flags);

		m_ignition->m_ignition = pts;
		m_ignition->m_ignitionPolyType = ignition_type;
		m_ignitionList.AddTail(m_ignition);
		i = m_ignitionList.NodeIndex(m_ignition);
		if (index)
			*index = i;

		m_bRequiresSave = true;
		return S_OK;
	} catch (...) {
		if (pts)
			delete pts;
		if (m_ignition)
			delete m_ignition;
	}
	return E_INVALIDARG;
}


HRESULT CCWFGM_Ignition::SetIgnition(std::uint32_t index, std::uint16_t ignition_type, const XY_PolyConst &xy_pairs) {
	SEM_BOOL engaged;
	CRWThreadSemaphoreEngage engage(m_lock, SEM_TRUE, &engaged, 1000000LL);
	if (!engaged)									return ERROR_SCENARIO_SIMULATION_RUNNING;

	if ((ignition_type != CWFGM_FIRE_IGNITION_POINT) && (xy_pairs.NumPoints() == 1)) {
    #ifdef DEBUG
											weak_assert(false);
    #endif
											return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
	}

	if ((ignition_type == CWFGM_FIRE_IGNITION_LINE) && (xy_pairs.NumPoints() < 2))		return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
	if ((ignition_type == CWFGM_FIRE_IGNITION_POLYGON_IN) && (xy_pairs.NumPoints() < 3))	return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
	if ((ignition_type == CWFGM_FIRE_IGNITION_POLYGON_OUT) && (xy_pairs.NumPoints() < 3))	return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;

	if (xy_pairs.NumPoints() == 0)								return E_INVALIDARG;

	std::uint16_t flags;
	if ((ignition_type == CWFGM_FIRE_IGNITION_POLYGON_IN) || (ignition_type == CWFGM_FIRE_IGNITION_POLYGON_OUT))		flags = XY_Poly::PolygonType::POLYGON;
	else if (ignition_type == CWFGM_FIRE_IGNITION_LINE)																	flags = XY_Poly::PolygonType::POLYLINE;
	else if (ignition_type == CWFGM_FIRE_IGNITION_POINT)																flags = XY_Poly::PolygonType::MULTIPOINT;
	else {
		weak_assert(false);
		return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
	}
	Ignition *m_ignition = m_ignitionList.IndexNode(index);
	bool to_add = false;
	if (!m_ignition) {
		if (index == m_ignitionList.GetCount()) {
			m_ignition = new Ignition();
			to_add = true;
		}
	}

	if (!m_ignition)
		return E_OUTOFMEMORY;

	XY_Poly *pts = nullptr;
	try {
		pts = new XY_Poly(xy_pairs);

		pts->CleanPoly(0.0, flags);

		if (m_ignition->m_ignition)
			delete m_ignition->m_ignition;
		m_ignition->m_ignition = pts;
		m_ignition->m_ignitionPolyType = ignition_type;
		if (to_add)
			m_ignitionList.AddTail(m_ignition);

		m_bRequiresSave = true;
		return S_OK;
	} catch (...) {
		if (pts)
			delete pts;
		if (to_add)
			delete m_ignition;
	}
	return E_INVALIDARG;
}


HRESULT CCWFGM_Ignition::SetIgnitionWKB(std::uint32_t index, const unsigned char *wkb) {
	SEM_BOOL engaged;
	CRWThreadSemaphoreEngage engage(m_lock, SEM_TRUE, &engaged, 1000000LL);
	if (!engaged)									return ERROR_SCENARIO_SIMULATION_RUNNING;

	Ignition *m_ignition = m_ignitionList.IndexNode(index);
	bool to_add = false;
	if (!m_ignition) {
		if (index == m_ignitionList.GetCount()) {
			m_ignition = new Ignition();
			to_add = true;
		}
	}

	if (!m_ignition)
		return E_OUTOFMEMORY;

	XY_Poly *pts = nullptr;
	try {
		pts = new XY_Poly();
		std::uint8_t type;
		if (!pts->LoadFromWKB(wkb, -1, nullptr, nullptr, &type)) {
			if (to_add)
				delete m_ignition;
			delete pts;
			return E_INVALIDARG;
		}

		if (type == XY_PolyConstTempl<double>::PolygonType::MULTIPOINT)
			m_ignition->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POINT;
		else if (type == XY_PolyConstTempl<double>::PolygonType::POLYLINE)
			m_ignition->m_ignitionPolyType = CWFGM_FIRE_IGNITION_LINE;
		else if (type == XY_PolyConstTempl<double>::PolygonType::POLYGON)
			m_ignition->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POLYGON_OUT;
		else
			m_ignition->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POLYGON_IN;

		if (m_ignition->m_ignition)
			delete m_ignition->m_ignition;
		m_ignition->m_ignition = pts;
		if (to_add)
			m_ignitionList.AddTail(m_ignition);

		m_bRequiresSave = true;
		return S_OK;
	}
	catch (...) {
		if (pts)
			delete pts;
		if (to_add)
			delete m_ignition;
	}
	return E_INVALIDARG;
}


HRESULT CCWFGM_Ignition::ClearIgnition(std::uint32_t index) {
	SEM_BOOL engaged;
	CRWThreadSemaphoreEngage engage(m_lock, SEM_TRUE, &engaged, 1000000LL);
	if (!engaged)									return ERROR_SCENARIO_SIMULATION_RUNNING;

	if (index == (std::uint32_t)-1) {
		Ignition *ig;
		while (ig = m_ignitionList.RemHead())
			delete ig;
	} else {
		if (index >= m_ignitionList.GetCount())						return E_INVALIDARG;

		Ignition *m_ignition = m_ignitionList.IndexNode(index);
		m_ignitionList.Remove(m_ignition);
		delete m_ignition;
	}
	m_bRequiresSave = true;
	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnition(std::uint32_t index, std::uint16_t *ignition_type, XY_Poly *xy_pairs) {
	if (!ignition_type)								return E_POINTER;
	if (!xy_pairs)									return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, SEM_FALSE);

	if (index >= m_ignitionList.GetCount())						return E_INVALIDARG;
	Ignition *m_ignition = m_ignitionList.IndexNode(index);

	xy_pairs->SetNumPoints(m_ignition->m_ignition->NumPoints());

	for (uint32_t i = 0; i < m_ignition->m_ignition->NumPoints(); i++)
		xy_pairs->SetPoint(i, m_ignition->m_ignition->GetPoint(i));
	*ignition_type = m_ignition->m_ignitionPolyType;

	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionLength(std::uint32_t index, double* length) {
	if (!length)									return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, FALSE);

	if (index >= m_ignitionList.GetCount())						return E_INVALIDARG;
	Ignition* m_ignition = m_ignitionList.IndexNode(index);

	if (m_ignition->m_ignitionPolyType == CWFGM_FIRE_IGNITION_POINT) {
		*length = 0;
		return S_OK;
	}
	else {
		if (m_ignition->m_ignitionPolyType == CWFGM_FIRE_IGNITION_LINE)
			*length = 0;
		else
			*length = m_ignition->m_ignition->GetPoint(0).DistanceTo(m_ignition->m_ignition->GetPoint(m_ignition->m_ignition->NumPoints() - 1));
		for (ULONG i = 0; i < (m_ignition->m_ignition->NumPoints() - 1); i++)
			*length += m_ignition->m_ignition->GetPoint(i).DistanceTo(m_ignition->m_ignition->GetPoint(i + 1));
	}
	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionCentroid(std::uint32_t index, XY_Point* xy) {
	CRWThreadSemaphoreEngage engage(m_lock, FALSE);

	if (index >= m_ignitionList.GetCount())						return E_INVALIDARG;
	Ignition* m_ignition = m_ignitionList.IndexNode(index);

	if ((m_ignition->m_ignitionPolyType == CWFGM_FIRE_IGNITION_POINT) ||
		(m_ignition->m_ignitionPolyType == CWFGM_FIRE_IGNITION_LINE)) {
		return (m_ignition->m_ignition->Average(*xy)) ? S_OK : E_FAIL;
	}
	else {
		return (m_ignition->m_ignition->Centroid(*xy)) ? S_OK : E_FAIL;
	}
}


HRESULT CCWFGM_Ignition::GetIgnitionRange(std::uint32_t index, XY_Point* min_pt, XY_Point* max_pt) {
	if (!min_pt)									return E_POINTER;
	if (!max_pt)									return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, SEM_FALSE);

	bool one = false;
	XY_Rectangle bbox;
	if (index != (std::uint32_t)-1) {
		if (index >= m_ignitionList.GetCount())					return E_INVALIDARG;
		Ignition *m_ignition = m_ignitionList.IndexNode(index);
		one = m_ignition->m_ignition->BoundingBox(bbox);
	} else {
		Ignition *ignition = m_ignitionList.LH_Head();
		while (ignition->LN_Succ()) {
			XY_Rectangle bb;
			if (ignition->m_ignition->BoundingBox(bb)) {
				if (!one) {
					one = true;
					bbox = bb;
				} else
					bbox.EncompassRectangle(bb);
			}
			ignition = ignition->LN_Succ();
		}
	}

	if (one) {
		min_pt->x = bbox.m_min.x;
		min_pt->y = bbox.m_min.y;
		max_pt->x = bbox.m_max.x;
		max_pt->y = bbox.m_max.y;
		return S_OK;
	}
	return ERROR_NO_DATA | ERROR_SEVERITY_WARNING;
}


HRESULT CCWFGM_Ignition::GetIgnitionWKB(std::uint32_t index, unsigned char **wkb) {
	if (!wkb || (*wkb))
		return E_POINTER;
	CRWThreadSemaphoreEngage engage(m_lock, SEM_FALSE);

	if (index >= m_ignitionList.GetCount())						return E_INVALIDARG;

	Ignition *m_ignition = m_ignitionList.IndexNode(index);
	
	uint64_t size;
	HRESULT hr;
	if (FAILED(hr = GetIgnitionWKBSize(index, &size)))
		return hr;
	unsigned char *wkb_c = (unsigned char *)calloc(size, sizeof(unsigned char));
	if (!m_ignition->m_ignition->StoreToWKB(&wkb_c, &size, nullptr, nullptr, 0)) return E_INVALIDARG;
	*wkb = wkb_c;
	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionType(std::uint32_t index, std::uint16_t *ignition_type) {
	if (!ignition_type)								return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, SEM_FALSE);

	if (index >= m_ignitionList.GetCount())						return E_INVALIDARG;
	Ignition *m_ignition = m_ignitionList.IndexNode(index);
	*ignition_type = m_ignition->m_ignitionPolyType;
	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionAttributeCount(std::uint32_t* count) {
	if (!count)									return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, FALSE);

	*count = m_attributeNames.size();
	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionAttributeName(std::uint32_t count, std::string* attribute_name) {
	if (!attribute_name)									return E_POINTER;
	if (count >= (std::uint32_t)m_attributeNames.size())	return E_INVALIDARG;

	std::set<std::string>::iterator it = m_attributeNames.begin();
	std::advance(it, count);
	std::string name(it->c_str());
	*attribute_name = name;
	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionAttributeValue(std::uint32_t index, const std::string& attribute_name, GDALVariant* value) {
	if (!value)								return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, FALSE);

	if (index >= m_ignitionList.GetCount())						return E_INVALIDARG;
	Ignition* m_ignition = m_ignitionList.IndexNode(index);

	for (auto a : m_ignition->m_attributes) {
		if (!stricmp(a.attributeName.c_str(), attribute_name.c_str())) {
			*value = a.attributeValue;
			return S_OK;
		}
	}
	return E_INVALIDARG;
}


HRESULT CCWFGM_Ignition::GetIgnitionSize(std::uint32_t index, std::uint32_t *ignition_size) {
	if (!ignition_size)								return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, SEM_FALSE);

	if (index == (std::uint32_t)-1) {
		*ignition_size = 0;
		Ignition *ig = m_ignitionList.LH_Head();
		while (ig->LN_Succ()) {
			if ((*ignition_size) < ig->m_ignition->NumPoints())
				*ignition_size = ig->m_ignition->NumPoints();
			ig = ig->LN_Succ();
		}
	} else {
		if (index >= m_ignitionList.GetCount())					return E_INVALIDARG;
		Ignition *m_ignition = m_ignitionList.IndexNode(index);
		*ignition_size = m_ignition->m_ignition->NumPoints();
	}
	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionWKBSize(std::uint32_t index, std::uint64_t *ignition_size) {
	if (!ignition_size)
		return E_POINTER;
	CRWThreadSemaphoreEngage engage(m_lock, SEM_FALSE);
	if (index == (std::uint32_t)-1){
		*ignition_size = 0;
		Ignition *ig = m_ignitionList.LH_Head();
		while (ig->LN_Succ()) {
			uint64_t size = ig->m_ignition->SizeofWKB();
			if ((*ignition_size) < size)
				*ignition_size = size;
			ig = ig->LN_Succ();
		}
	} else {
		if (index >= m_ignitionList.GetCount())		return E_INVALIDARG;
		*ignition_size =  m_ignitionList.IndexNode(index)->m_ignition->SizeofWKB();
	}

	return S_OK;
}


HRESULT CCWFGM_Ignition::GetIgnitionCount(std::uint32_t *count) {
	if (!count)									return E_POINTER;

	CRWThreadSemaphoreEngage engage(m_lock, SEM_FALSE);

	*count = m_ignitionList.GetCount();
	return S_OK;
}


HRESULT CCWFGM_Ignition::SetAttribute(std::uint16_t option, const PolymorphicAttribute &var) {
	boost::intrusive_ptr<ICWFGM_GridEngine> gridEngine = m_gridEngine;
	if (!gridEngine)							{ weak_assert(false); return ERROR_GRID_UNINITIALIZED; }

	switch (option) {
		case CWFGM_GRID_ATTRIBUTE_GIS_URL: {
								std::string str;
								try {
									str = std::get<std::string>(var);
								}
								catch (std::bad_variant_access &) {
									weak_assert(false);
									break;
								}
								if (str.length()) {
									m_gisURL = std::move(str);
									m_bRequiresSave = true;
								}
								}
								return S_OK;

		case CWFGM_GRID_ATTRIBUTE_GIS_LAYER: {
								std::string str;
								try {
									str = std::get<std::string>(var);
								}
								catch (std::bad_variant_access &) {
									weak_assert(false);
									break;
								}
								if (str.length()) {
									m_gisLayer = std::move(str);
									m_bRequiresSave = true;
								}
								}
								return S_OK;

		case CWFGM_GRID_ATTRIBUTE_GIS_UID: {
								std::string str;
								try {
									str = std::get<std::string>(var);
								}
								catch (std::bad_variant_access &) {
									weak_assert(false);
									break;
								}
								m_gisUID = std::move(str);
								m_bRequiresSave = true;
								}
								return S_OK;

		case CWFGM_GRID_ATTRIBUTE_GIS_PWD: {
								std::string str;
								try {
									str = std::get<std::string>(var);
								}
								catch (std::bad_variant_access &) {
									weak_assert(false);
									break;
								}
								m_gisPWD = (str);
								m_bRequiresSave = true;
								}
								return S_OK;
	}

	return E_INVALIDARG;
}


HRESULT CCWFGM_Ignition::GetAttribute(std::uint16_t option, PolymorphicAttribute *value) {
	if (!value)									return E_POINTER;

	switch (option) {
		case CWFGM_ATTRIBUTE_LOAD_WARNING: {
							*value = m_loadWarning;
							return S_OK;
						   }
		case CWFGM_GRID_ATTRIBUTE_GIS_URL: {
							*value = m_gisURL;
							return S_OK;
						   }
		case CWFGM_GRID_ATTRIBUTE_GIS_LAYER: {
							*value = m_gisLayer;
							return S_OK;
						   }
		case CWFGM_GRID_ATTRIBUTE_GIS_UID: {
							*value = m_gisUID;
							return S_OK;
						   }
		case CWFGM_GRID_ATTRIBUTE_GIS_PWD: {
							*value = m_gisPWD;
							return S_OK;
						   }
	}
	return E_INVALIDARG;
}
