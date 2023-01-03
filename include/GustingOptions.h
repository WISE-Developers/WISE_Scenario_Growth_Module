/**
 * WISE_Scenario_Growth_Module: GustingOptions.h
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

#include "ISerializeProto.h"
#include "cwfgmScenario.pb.h"
#include "WTime.h"
#include <string>

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 4)
#endif

template<class _type>
class Scenario;

template<class _type>
class ScenarioFire;

template<class _type>
class GustingOptions : public ISerializeProto {
protected:
	std::uint32_t	m_gustingMode = 0;					// 0 means no gusting
														// 1 means Cordy's approach - just average together gusting and regular wind speed
	std::uint32_t	m_gustsPerHour = 0;
	double			m_percentGusting = 0.0;
	std::int32_t	m_gustingBias = 0;					// -1 means start of period, 0 means center of period, 1 means end of period
	bool			m_bRequiresSave = false;

	double calculateGustPercent(const ScenarioFire<_type>* sf, WTimeSpan &numerator, WTimeSpan &denominator) const;

public:
	double ApplyGusting(const ScenarioFire<_type>* sts, const HSS_Time::WTime& time, const double windSpeed, const double windGusting) const;
	HRESULT GetEventTime(const Scenario<_type> *scenario, std::uint32_t flags,
		const HSS_Time::WTime& from_time, HSS_Time::WTime* next_event);
	double PercentGusting(const ScenarioFire<_type>* sts, const HSS_Time::WTime& time) const;
	double AssignPercentGusting(const ScenarioFire<_type>* sts, const HSS_Time::WTime& time) const;

	virtual std::int32_t serialVersionUid(const SerializeProtoOptions& options) const noexcept override;
	virtual WISE::FireEngineProto::CwfgmScenario::GustingOptions* serialize(const SerializeProtoOptions& options) override;
	virtual GustingOptions<_type>* deserialize(const google::protobuf::Message& proto, std::shared_ptr<validation::validation_object> valid, const std::string& name) override;
	virtual std::optional<bool> isdirty(void) const noexcept override { return m_bRequiresSave; }

	std::string		m_loadWarning;
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif
