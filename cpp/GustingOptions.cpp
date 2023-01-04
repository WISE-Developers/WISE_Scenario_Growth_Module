/**
 * WISE_Scenario_Growth_Module: GustingOptions.cpp
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

#include "types.h"
#include "ICWFGM_Fuel.h"
#include "ScenarioTimeStep.h"
#include "results.h"

template<class _type>
double GustingOptions<_type>::ApplyGusting(const ScenarioFire<_type>* sf, const HSS_Time::WTime& time, const double windSpeed, const double windGusting) const {
	return windSpeed * (1.0 - sf->m_gusting) + (windGusting * sf->m_gusting);
}


template<class _type>
double GustingOptions<_type>::AssignPercentGusting(const ScenarioFire<_type>* sf, const HSS_Time::WTime& time) const {
	((ScenarioFire<_type>*)sf)->m_gusting = PercentGusting(sf, time);
	return sf->m_gusting;
}


template<class _type>
double GustingOptions<_type>::PercentGusting(const ScenarioFire<_type>* sf, const HSS_Time::WTime& time) const {
	if (m_gustingMode == 0)
		return 0.0;
	if (m_gustingMode == 1)
		return m_percentGusting;
	if (m_gustingMode == 2) {
		if (m_gustsPerHour < 1)
			return 0.0;

		HSS_Time::WTimeSpan duration(60 * 60);		// 1 hour;
		duration /= (INTNM::int32_t)m_gustsPerHour;															// how long each cycle is as fraction of an hour
		HSS_Time::WTimeSpan gust_duration(duration);
		gust_duration *= m_percentGusting;																// how long each gusting period is as fraction of an hour

		HSS_Time::WTime htime(time);
		htime.PurgeToHour(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
		HSS_Time::WTimeSpan hour_percent = time - htime;
		HSS_Time::WTimeSpan part_of_duration(hour_percent % duration);

		if (m_gustingBias < 0) {
			if (part_of_duration < gust_duration)
				return 1.0;
			return 0.0;
		}
		else if (m_gustingBias > 0) {
			if (part_of_duration < (duration - gust_duration))
				return 0.0;
			return 1.0;
		}
		if (part_of_duration < ((duration - gust_duration) / 2))
			return 0.0;
		if (part_of_duration < ((duration + gust_duration) / 2))
			return 1.0;
		return 0.0;
	}
	if (m_gustingMode == 3) {
		WTimeSpan numerator, denominator;
		double percentage = calculateGustPercent(sf, numerator, denominator);

		bool should_gust;
		if (percentage < 0.0) {						// or... this is the first timestep
			if (m_gustingBias < 0)
				should_gust = true;
			else
				should_gust = false;
		}
		else {
			should_gust = (sf->m_gusting == 0.0);	// alternate between past gusting
			if (should_gust) {
				if (percentage > m_percentGusting)
					should_gust = false;
			}
			else {
				if (percentage < m_percentGusting)
					should_gust = true;
			}
		}

		if (should_gust)
			return 1.0;
		return 0.0;
	}
	return -1.0;
}


template<class _type>
double GustingOptions<_type>::calculateGustPercent(const ScenarioFire<_type>* sf, WTimeSpan& numerator, WTimeSpan& denominator) const {
																			// this is called once to see if we could/should gust to get event times,
																			// then later when we know the duration of the current time step
	numerator = WTimeSpan(0);
	denominator = WTimeSpan(0);
	if (!sf)
		return -1.0;
	const ScenarioFire<_type>* psf = sf->LN_CalcPred();
	WTime go_hr(sf->TimeStep()->m_time), min_hr(sf->TimeStep()->m_time);
	go_hr.PurgeToHour(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	min_hr -= WTimeSpan(0, 1, 0, 0);
	if (go_hr == sf->TimeStep()->m_time)
		return -1.0;
	while (psf) {
		WTimeSpan duration = sf->TimeStep()->m_time - psf->TimeStep()->m_time;
		if (psf->m_gusting != 0.0)
			numerator += duration * psf->m_gusting;
		denominator += duration;
		if (psf->TimeStep()->m_time == go_hr)
			break;
		if (psf->TimeStep()->m_time <= min_hr)
			break;
		sf = psf;
		psf = psf->LN_CalcPred();
	}
	if (!denominator.GetTotalMicroSeconds())
		return -1.0;							// no time steps at all
	return numerator / denominator;
}


template<class _type>
HRESULT GustingOptions<_type>::GetEventTime(const Scenario<_type>* scenario, std::uint32_t flags,
	   const HSS_Time::WTime& from_time, HSS_Time::WTime* next_event) {
	if (m_gustingMode == 2) {
		if (m_gustsPerHour < 1.0)
			return S_OK;

		HSS_Time::WTimeSpan duration(60 * 60);		// 1 hour;
		duration /= (INTNM::int32_t)m_gustsPerHour;															// how long each cycle is as fraction of an hour
		HSS_Time::WTimeSpan gust_duration(duration);
		gust_duration *= m_percentGusting;																// how long each gusting period is as fraction of an hour

		HSS_Time::WTime htime(from_time);
		htime.PurgeToHour(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
		HSS_Time::WTimeSpan hour_percent = from_time - htime;
		HSS_Time::WTimeSpan part_of_duration(hour_percent % duration);
		std::int32_t start = (std::int32_t)floor(hour_percent / duration);

		HSS_Time::WTimeSpan ws_event;

		if (m_gustingBias < 0) {
			if (part_of_duration < gust_duration)
				ws_event = duration * start + gust_duration;
			else
				ws_event = duration * (start + 1);
		}
		else if (m_gustingBias > 0) {
			if (part_of_duration < (duration - gust_duration))
				ws_event = duration * (start + 1) - gust_duration;
			else
				ws_event = duration * (start + 1);
		}
		else {
			if (part_of_duration < ((duration - gust_duration) / 2))
				ws_event = duration * start + (duration - gust_duration) / 2;
			else if (part_of_duration < ((duration + gust_duration) / 2))
				ws_event = duration * start + (duration + gust_duration) / 2;
			else
				ws_event = duration * (start + 1) + (duration - gust_duration) / 2;
		}

		HSS_Time::WTime wt_event(from_time);
		wt_event.PurgeToHour(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
		wt_event += ws_event;

		if (wt_event < (*next_event))
			*next_event = wt_event;

		return S_OK;
	}
	if (m_gustingMode == 3) {
		ActiveFire<_type>* af = scenario->m_activeFires.LH_Head();
		while (af->LN_Succ()) {
			WTimeSpan numerator, denominator;
			double percentage = calculateGustPercent(af->LN_Ptr(), numerator, denominator);
			WTime start_hour(from_time);
			start_hour.PurgeToHour(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
			WTime next_hour(start_hour + WTimeSpan(0, 1, 0, 0));
			WTimeSpan remaining = next_hour - from_time;
			WTimeSpan max_gust_remaining = (remaining + denominator) * m_percentGusting - numerator;

			bool should_gust;
			if (percentage == -1.0) {					// or... this is the first timestep
				if (m_gustingBias < 0)
					should_gust = true;
				else
					should_gust = false;
			}
			else {
				should_gust = (af->LN_Ptr()->m_gusting == 0.0);	// alternate between past gusting
				if (should_gust) {
					if (percentage > m_percentGusting)
						should_gust = false;
				}
				else {
					if (percentage < m_percentGusting)
						should_gust = true;
				}
			}

			if (should_gust) {
				if ((from_time + max_gust_remaining) < (*next_event))
					*next_event = from_time + max_gust_remaining;
			}
			else {
				if ((next_hour - max_gust_remaining) < (*next_event))
					*next_event = next_hour - max_gust_remaining;
			}
			af = af->LN_Succ();
		}
	}
	return S_OK;
}


template<class _type>
std::int32_t GustingOptions<_type>::serialVersionUid(const SerializeProtoOptions& options) const noexcept {
	return 1;
}


template<class _type>
WISE::FireEngineProto::CwfgmScenario::GustingOptions* GustingOptions<_type>::serialize(const SerializeProtoOptions& options) {
	auto gusting = new WISE::FireEngineProto::CwfgmScenario::GustingOptions();
	gusting->set_version(serialVersionUid(options));
	gusting->set_gusting((WISE::FireEngineProto::CwfgmScenario_GustingOptions_Gusting)m_gustingMode);
	if (m_gustingMode > 1) {
		gusting->set_gustsperhour(m_gustsPerHour);
		gusting->set_percentgusting(m_percentGusting * 100.0);
		gusting->set_gustbias((WISE::FireEngineProto::CwfgmScenario_GustingOptions_GustingBias)m_gustingBias);
	}
	return gusting;
}


template<class _type>
GustingOptions<_type>* GustingOptions<_type>::deserialize(const google::protobuf::Message& proto, std::shared_ptr<validation::validation_object> valid, const std::string& name) {
	auto gusting = dynamic_cast_assert<const WISE::FireEngineProto::CwfgmScenario::GustingOptions*>(&proto);

	if (!gusting)
	{
		if (valid)
			/// <summary>
			/// The object passed as a scenario is invalid. An incorrect object type was passed to the parser.
			/// </summary>
			/// <type>internal</type>
			valid->add_child_validation("WISE.FireEngineProto.CwfgmScenario.GustingOptions", name, validation::error_level::SEVERE, validation::id::object_invalid, proto.GetDescriptor()->name());
		weak_assert(false);
		throw ISerializeProto::DeserializeError("WISE.FireEngineProto.CwfgmScenario.GustingOptions: Protobuf object invalid", ERROR_PROTOBUF_OBJECT_INVALID);
	}
	if (gusting->version() != 1)
	{
		if (valid)
			/// <summary>
			/// The object version is not supported. The scenario is not supported by this version of Prometheus.
			/// </summary>
			/// <type>user</type>
			valid->add_child_validation("WISE.FireEngineProto.CwfgmScenario.GustingOptions", name, validation::error_level::SEVERE, validation::id::version_mismatch, std::to_string(gusting->version()));
		weak_assert(false);
		throw ISerializeProto::DeserializeError("WISE.FireEngineProto.CwfgmScenario.GustingOptions: Version is invalid", ERROR_PROTOBUF_OBJECT_VERSION_INVALID);
	}

	std::string throw_msg;
	auto vt = validation::conditional_make_object(valid, "WISE.FireEngineProto.CwfgmScenario", name);
	auto v = vt.lock();

	m_gustingMode = (std::uint32_t)gusting->gusting();

	if (gusting->has_gustsperhour()) {
		m_gustsPerHour = (std::uint32_t)gusting->gustsperhour();

		if (m_gustsPerHour < 0)
		{
			m_loadWarning = "Warning: CWFGM_Scenario.GustingOptions.gustsPerHour: Invalid gusting percentage";
			if (v)
				/// <summary>
				/// The scenario's gusting percentage must be between 0 and 100, inclusive.
				/// </summary>
				/// <type>user</type>
				v->add_child_validation("int32", "gustsPerHour", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(m_percentGusting), { true, 0 }, { true, 60 });
			m_gustsPerHour = 0;
		}
		else if (m_percentGusting > 60)
		{
			m_loadWarning = "Warning: CWFGM_Scenario: Invalid initial vertex count";
			if (v)
				/// <summary>
				/// The scenario's initial vertex count must be between 6 and 64, inclusive.
				/// </summary>
				/// <type>user</type>
				v->add_child_validation("int32", "gustsPerHour", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(m_percentGusting), { true, 0 }, { true, 60 });
			m_gustsPerHour = 60;
		}
	}

	if (gusting->has_percentgusting())
	{
		m_percentGusting = gusting->percentgusting();

		if (m_percentGusting < 0.0)
		{
			m_loadWarning = "Warning: CWFGM_Scenario.GustingOptions.percentGusting: Invalid gusting percentage";
			if (v)
				/// <summary>
				/// The scenario's gusting percentage must be between 0 and 100, inclusive.
				/// </summary>
				/// <type>user</type>
				v->add_child_validation("double", "percentGusting", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(m_percentGusting), { true, 0.0 }, { true, 100.0 });
			m_percentGusting = 0.0;
		}
		else if (m_percentGusting > 100.0)
		{
			m_loadWarning = "Warning: CWFGM_Scenario.GustingOptions.percentGusting: Invalid gusting percentage";
			if (v)
				/// <summary>
				/// The scenario's initial vertex count must be between 6 and 64, inclusive.
				/// </summary>
				/// <type>user</type>
				v->add_child_validation("double", "percentGusting", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(m_percentGusting), { true, 0.0 }, { true, 100.0 });
			m_percentGusting = 100.0;
		}

		m_percentGusting /= 100.0;
	}

	if (gusting->has_gustbias())
		m_gustingBias = (std::int32_t)gusting->gustbias();

	return this;
}

#include "InstantiateClasses.cpp"
