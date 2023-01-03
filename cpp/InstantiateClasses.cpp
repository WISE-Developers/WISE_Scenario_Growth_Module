/**
 * WISE_Scenario_Growth_Module: InstantiateClasses.cpp
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

template class Scenario<fireengine_float_type>;
template class FirePoint<fireengine_float_type>;
template class FireFront<fireengine_float_type>;
template class ActiveFire<fireengine_float_type>;
template class ScenarioFire<fireengine_float_type>;
template class IgnitionNode<fireengine_float_type>;
template class ScenarioCache<fireengine_float_type>;
template class XY_PolyLLSetBB<fireengine_float_type>;
template class FireFrontStats<fireengine_float_type>;
template class FireFrontExport<fireengine_float_type>;
template struct growVoxelParms<fireengine_float_type>;
template struct growPointStruct<fireengine_float_type>;
template class ScenarioTimeStep<fireengine_float_type>;
template class ScenarioGridCache<fireengine_float_type>;
template struct growVoxelIterator<fireengine_float_type>;
template class ScenarioFireExport<fireengine_float_type>;
template class GustingOptions<fireengine_float_type>;
