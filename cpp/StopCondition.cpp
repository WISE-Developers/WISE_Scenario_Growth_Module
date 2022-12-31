/**
 * WISE_Scenario_Growth_Module: StopCondition.cpp
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

#include "StopCondition.h"


StopCondition::StopCondition() {
	responseTime = WTimeSpan(0, 0, 0, 0);

	fi90PercentThreshold = -1.0;
	fi95PercentThreshold = -1.0;
	fi100PercentThreshold = -1.0;
	RHThreshold = -1.0;
	PrecipThreshold = -1.0;
	areaThreshold = -1.0;
	burnDistanceThreshold = -1.0;

	fi90PercentDuration = WTimeSpan(0, 6, 0, 0);
	fi95PercentDuration = WTimeSpan(0, 6, 0, 0);
	fi100PercentDuration = WTimeSpan(0, 6, 0, 0);
	RHDuration = WTimeSpan(0, 6, 0, 0);
	PrecipDuration = WTimeSpan(5, 0, 0, 0);
}


bool StopCondition::anythingValid() const {
	return (fi90 || fi95 || fi100 || RH || precip || area || burnDistance);
}
