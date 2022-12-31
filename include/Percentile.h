/**
 * WISE_Scenario_Growth_Module: Percentile.h
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

#pragma once

#ifdef MSVC_COMPILER
#pragma managed(push, off)
#endif

#include "FuelCom.h"
#include "hssconfig/config.h"
#include <vector>

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif

struct ScenarioPercentileEntry {
public:
	const _GUID *defaultFuelType;
	double surface_s, crown_s;
};


struct ScenarioFuelName {
public:
	const _GUID *fuel;
	const char *text;
};

class ScenarioPercentile
{
	const ScenarioFuelName *validFuels;
	std::vector<ScenarioPercentileEntry> percentile;

public:
	ScenarioPercentile();
	ScenarioPercentile(const ScenarioPercentile &toCopy);

	HRESULT GetPercentileClassCount(std::uint8_t *count) const;
	HRESULT GetPercentile(unsigned char indexFuel, _GUID *defaultFuel) const;
	HRESULT GetPercentile(const char* name, _GUID *defaultFuel) const;
	HRESULT GetPercentileName(const _GUID *defaultFuel, char* name, size_t size) const;
	HRESULT SetPercentileValue(const _GUID *defaultFuel, unsigned char fireDescription, double s);
	HRESULT GetPercentileValue(const _GUID *defaultFuel, unsigned char fireDescription, double *s) const;
	HRESULT IsPercentileDefault(const _GUID *defaultFuel, bool* b) const;
	HRESULT RSI(double tinv, const _GUID *clsId, double RSIin, double CFBin, double *RSIout);
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif

#ifdef MSVC_COMPILER
#pragma managed(pop)
#endif
