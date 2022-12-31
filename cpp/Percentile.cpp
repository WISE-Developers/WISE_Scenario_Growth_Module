/**
 * WISE_Scenario_Growth_Module: Percentile.cpp
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
#include "guid.h"
#include <boost/algorithm/string.hpp>

#if __has_include(<mathimf.h>)
#include <mathimf.h>
#else
#include <cmath>
#endif

#include "ICWFGM_FBPFuel.h"
#include "Percentile.h"
#include "math_constants.h"

//the protobuf export depends on these defaults not changing,
//so don't change them without updating the serialization
static const ScenarioPercentileEntry defaultEntries[] = {
	{ &CLSID_CWFGM_Fuel_C1, -1.0, 0.95 },
	{ &CLSID_CWFGM_Fuel_C2, 0.84, 1.82 },
	{ &CLSID_CWFGM_Fuel_C3, 0.62, 1.78 },
	{ &CLSID_CWFGM_Fuel_C4, 0.74, 1.38 },
	{ &CLSID_CWFGM_Fuel_C5, 0.8, -1.0 },
	{ &CLSID_CWFGM_Fuel_C6, 0.66, 1.54 },
	{ &CLSID_CWFGM_Fuel_C7, 1.22, 1.0 },
	{ &CLSID_CWFGM_Fuel_D1, 0.716, -1.0 },
	{ &CLSID_CWFGM_Fuel_M3, 0.551, -1.0 },
	{ NULL, -1.0, -1.0 }
};

//the protobuf export depends on these defaults not changing,
//so don't change them without updating the serialization
static const ScenarioFuelName sValidFuels[] = {
	{ &CLSID_CWFGM_Fuel_C1, _T("C-1") },
	{ &CLSID_CWFGM_Fuel_C2, _T("C-2") },
	{ &CLSID_CWFGM_Fuel_C3, _T("C-3") },
	{ &CLSID_CWFGM_Fuel_C4, _T("C-4") },
	{ &CLSID_CWFGM_Fuel_C5, _T("C-5") },
	{ &CLSID_CWFGM_Fuel_C6, _T("C-6") },
	{ &CLSID_CWFGM_Fuel_C7, _T("C-7") },
	{ &CLSID_CWFGM_Fuel_D1, _T("D-1") },
	{ &CLSID_CWFGM_Fuel_D2, _T("D-2") },
	{ &CLSID_CWFGM_Fuel_D1D2, _T("D-1/D-2") },
	{ &CLSID_CWFGM_Fuel_M1, _T("M-1") },
	{ &CLSID_CWFGM_Fuel_M2, _T("M-2") },
	{ &CLSID_CWFGM_Fuel_M1M2, _T("M-1/M-2") },
	{ &CLSID_CWFGM_Fuel_M3, _T("M-3") },
	{ &CLSID_CWFGM_Fuel_M4, _T("M-4") },
	{ &CLSID_CWFGM_Fuel_M3M4, _T("M-3/M-4") },
	{ &CLSID_CWFGM_Fuel_S1, _T("S-1") },
	{ &CLSID_CWFGM_Fuel_S2, _T("S-2") },
	{ &CLSID_CWFGM_Fuel_S3, _T("S-3") },
	{ &CLSID_CWFGM_Fuel_O1a, _T("O-1a") },
	{ &CLSID_CWFGM_Fuel_O1b, _T("O-1b") },
	{ NULL, NULL }
};


ScenarioPercentile::ScenarioPercentile() {
	validFuels = sValidFuels;
	const ScenarioPercentileEntry *d = defaultEntries;
	while (d->defaultFuelType) {
		percentile.push_back(*d);
		d++;
	}
}


ScenarioPercentile::ScenarioPercentile(const ScenarioPercentile &toCopy) {
	validFuels = sValidFuels;
	for (std::vector<ScenarioPercentileEntry>::const_iterator it = toCopy.percentile.begin(); it != toCopy.percentile.end(); it++) {
		ScenarioPercentileEntry pe;
		pe.defaultFuelType = it->defaultFuelType;
		pe.crown_s = it->crown_s;
		pe.surface_s = it->surface_s;
		percentile.push_back(pe);
	}
}


HRESULT ScenarioPercentile::GetPercentileClassCount(std::uint8_t *count) const {
	*count = (std::uint8_t)percentile.size();
	return S_OK;
}


HRESULT ScenarioPercentile::GetPercentile(const char* name, _GUID *defaultFuel) const
{
	const ScenarioFuelName *c = validFuels;
	while (c->fuel)
	{
		if (boost::iequals(name, c->text))
		{
			memcpy(defaultFuel, c->fuel, sizeof(_GUID));
			return S_OK;
		}
		c++;
	}

	return E_INVALIDARG;
}


HRESULT ScenarioPercentile::GetPercentile(unsigned char indexFuel, _GUID *defaultFuel) const {
	if (indexFuel >= percentile.size())
		return E_INVALIDARG;
	memcpy(defaultFuel, percentile[indexFuel].defaultFuelType, sizeof(_GUID));
	return S_OK;
}


HRESULT ScenarioPercentile::SetPercentileValue(const _GUID *defaultFuel, unsigned char fireDescription, double s) {
	size_t i;
	ScenarioPercentileEntry *p = nullptr;
    bool set = false;
	for (i = 0; i < percentile.size(); i++) {
		if (!memcmp(defaultFuel, percentile[i].defaultFuelType, sizeof(GUID))) {
			p = &percentile[i];
            set = true;
            break;
		}
	}

	ScenarioPercentileEntry new_sp;
	if (!set)
    {
        const ScenarioFuelName *c = validFuels;
        while (c->fuel) {
            if ((*defaultFuel) == *(c->fuel)) {
                p = &new_sp;
                new_sp.defaultFuelType = c->fuel;
                new_sp.crown_s = -1.0;
                new_sp.surface_s = -1.0;
                set = true;
                break;
            }
            c++;
        }
        if (!set)
            return E_INVALIDARG;
    }

	switch (fireDescription) {
		case 0: p->crown_s = s; return S_OK;
		case 1: p->surface_s = s; return S_OK;
	}

	return E_INVALIDARG;
}


HRESULT ScenarioPercentile::GetPercentileName(const _GUID *defaultFuel, char* name, size_t size) const
{
	const ScenarioFuelName *c = validFuels;
	while (c->fuel)
	{
		if (!memcmp(defaultFuel, c->fuel, sizeof(_GUID)))
		{
#ifdef _MSC_VER
			strcpy_s(name, size, c->text);
#else
			strcpy(name, c->text);
#endif
			return S_OK;
		}
		c++;
	}

	return E_INVALIDARG;
}


HRESULT ScenarioPercentile::IsPercentileDefault(const _GUID *defaultFuel, bool* b) const
{
	*b = false;
	for (size_t i = 0; i < percentile.size(); i++)
	{
		if (!memcmp(defaultFuel, percentile[i].defaultFuelType, sizeof(_GUID)))
		{
			const ScenarioPercentileEntry *d = defaultEntries;
			while (d->defaultFuelType)
			{
				if (!memcmp(defaultFuel, d->defaultFuelType, sizeof(_GUID)))
				{
					if (d->crown_s == percentile[i].crown_s && d->surface_s == percentile[i].surface_s)
						*b = true;

					return S_OK;
				}
				d++;
			}

			return E_INVALIDARG;
		}
	}

	return E_INVALIDARG;
}


HRESULT ScenarioPercentile::GetPercentileValue(const _GUID *defaultFuel, unsigned char fireDescription, double *s) const
{
	for (size_t i = 0; i < percentile.size(); i++)
	{
		if (!memcmp(defaultFuel, percentile[i].defaultFuelType, sizeof(_GUID)))
		{
			switch (fireDescription)
			{
				case 0: *s = percentile[i].crown_s; return S_OK;
				case 1: *s = percentile[i].surface_s; return S_OK;
			}
			return E_INVALIDARG;
		}
	}

	return E_INVALIDARG;
}


HRESULT ScenarioPercentile::RSI(double tinv, const _GUID *clsId, double RSIin, double CFBin, double *RSIout) {
	for (size_t i = 0; i < percentile.size(); i++) {
		if (!memcmp(clsId, percentile[i].defaultFuelType, sizeof(_GUID))) {
			if (CFBin < 0.1) {
				if (percentile[i].surface_s < 0.0)
					break;
				*RSIout = pow(CONSTANTS_NAMESPACE::E<double>(), tinv) * RSIin;
				return S_OK;
			} else {
				if (percentile[i].crown_s < 0.0)
					break;
				double d = pow(RSIin, 0.6);
				double e = tinv * percentile[i].crown_s;
				if ((-e) > d)
					*RSIout = pow(CONSTANTS_NAMESPACE::E<double>(), tinv) * RSIin;
				else
					*RSIout = pow(d + e, (1.0 / 0.6));
				return S_OK;
			}
		}
	}
	*RSIout = RSIin;
	return S_OK;
}
