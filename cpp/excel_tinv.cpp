/**
 * WISE_Scenario_Growth_Module: excel_tinv.cpp
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

#include <cmath>

#include "excel_tinv.h"
#include <boost/math/distributions/students_t.hpp>


double tinv(double probability, int freedom) {

#ifdef _WINDOWS
	const wchar_t* c = L"", * d = L"";
	wmemcmp(c, d, 0);
#endif

	boost::math::students_t dist(freedom);
	return boost::math::quantile(dist, probability);
}