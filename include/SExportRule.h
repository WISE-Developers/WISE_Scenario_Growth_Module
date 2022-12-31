/**
 * WISE_Scenario_Growth_Module: SExportRule.h
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

#include "CWFGM_Fire.h"
#include "CWFGM_Asset.h"

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif

/** Fire export rules

	Structure defining various options for export SHP and KML files (like names, values, etc.).
*/
struct SExportRule {
	CCWFGM_Ignition *ignition;		/*! Identifies the ignition that the rule applies to, NULL for all (or no) ignitions */
	CCWFGM_Asset *asset;			/*! Identifies the asset that the rule applies to, NULL for all (or no) assets */
	std::uint32_t assetIndex;		/*! Identifies the index of the asset geometry that the rule applies to, NULL for all (or no) assets */
	std::uint32_t operation;		/*! how to interpret name, value */
	std::string name;				/*! Name of the attibute data field */
	PolymorphicAttribute value;		/*! Value of the attribute data field */
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif
