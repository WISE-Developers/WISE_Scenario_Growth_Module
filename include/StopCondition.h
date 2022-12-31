/**
 * WISE_Scenario_Growth_Module: StopCondition.h
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

#include <atomic>
#include "WTime.h"

using namespace HSS_Time;

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 4)
#endif

class StopCondition {
public:
	bool	fi90 = false,
			fi95 = false,						// whether or not the following variables are valid
			fi100 = false,
			RH = false,
		precip = false,
		area = false,
		burnDistance = false;

	WTimeSpan			responseTime;			// if this is set, then the below conditions won't take effect until after this has expired
												// a value of 0 (which means immediate response time)
	double				fi90PercentThreshold;
	WTimeSpan			fi90PercentDuration;

	double				fi95PercentThreshold;
	WTimeSpan			fi95PercentDuration;

	double				fi100PercentThreshold;
	WTimeSpan			fi100PercentDuration;

	double				RHThreshold;
	WTimeSpan			RHDuration;

	double				PrecipThreshold;
	WTimeSpan			PrecipDuration;

	double				areaThreshold;

	double				burnDistanceThreshold;

	bool anythingValid() const;

	StopCondition();
};


class StopConditionState {
public:
	std::atomic<bool>	fi90 = true,
						fi95 = true,					// whether or not the condition holds true.  e.g. if RH threshold is 55, this is true until RH climbs over 55 (meaning potential to stop the fire)
						fi100 = true,					// true if we can continue modelling
						RH = false,						// set up for spatial wx modelling, so long as one part of the perimeter is < threshold, then we'll continue burning
						precip = true,
						area = true,
						burnDistance = true;
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif
