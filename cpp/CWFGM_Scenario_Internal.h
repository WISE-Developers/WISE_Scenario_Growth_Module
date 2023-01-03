/**
 * WISE_Scenario_Growth_Module: CWFGM_Scenario_Internal.cpp
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

struct CCWFGM_Scenario::Impl
{
	MinListTempl<IgnitionNode<fireengine_float_type>>		m_ignitionList;
	MinListTempl<AssetNode<fireengine_float_type>>			m_assetList;
	GustingOptions<fireengine_float_type>					m_go;
	Scenario<fireengine_float_type>*						m_scenario{ nullptr };

	void unlockObjects(CCWFGM_Scenario& _this, VectorEngineNode* ve, AssetNode<fireengine_float_type>* an);
};
