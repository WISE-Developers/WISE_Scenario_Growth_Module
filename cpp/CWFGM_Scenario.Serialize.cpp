/**
 * WISE_Scenario_Growth_Module: CWFGM_Scenario.Serialize.cpp
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
#include "google/protobuf/message.h"
#include "angles.h"
#include "CWFGM_Scenario.h"
#include "FireEngine_ext.h"
#include "GridCom_ext.h"
#include "WeatherCom_ext.h"
#include "scenario.h"
#include "results.h"
#include "Percentile.h"
#include "ScenarioExportRules.h"
#include "str_printf.h"
#include "doubleBuilder.h"
#include "CWFGM_Scenario_Internal.h"

#include "guid.h"

#ifdef DEBUG
#include <assert.h>
#endif

#define TRUTH_FLAG(flags, index) (((flags) & (1ull << (index))) ? true : false)
#define FLAG_TRUTH(flags, index, truth) flags = (truth) ? ((flags) | (1ull << (index))) : ((flags) & ~(1ull << (index)))


HRESULT CCWFGM_Scenario::ExportFires(const CCWFGM_Ignition *set, HSS_Time::WTime &start_time, HSS_Time::WTime &end_time, std::uint16_t flags,
    const std::string &driver_name, const std::string &projection, const std::string &file_path, const ScenarioExportRules *rules) const {
	if ((!start_time.GetTotalMicroSeconds()) || (!end_time.GetTotalMicroSeconds()))			return ERROR_FIRE_INVALID_TIME;
	if (!driver_name.length())								return E_POINTER;
	if (!file_path.length())									return E_POINTER;

	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore &>(m_lock), SEM_FALSE);

	if (m_impl->m_scenario) {
		WTime st(start_time, m_timeManager), et(end_time, m_timeManager);
		ScenarioExportRules rr;
		HRESULT hr = m_impl->m_scenario->Export(set, &st, &et, flags, driver_name.c_str(), projection.c_str(), file_path.c_str(), *rules);
		if (SUCCEEDED(hr)) {
			start_time.SetTime(st);
			end_time.SetTime(et);
		}
		return hr;
	}
	return ERROR_SCENARIO_BAD_STATE;
}


HRESULT CCWFGM_Scenario::ExportCriticalPath(const ICWFGM_Asset* asset, const std::uint32_t index, const std::uint16_t flags,
	const std::string& driver_name, const std::string& projection, const std::string& file_path, const ScenarioExportRules* rules) const {
	if (!driver_name.length())									return E_POINTER;
	if (!file_path.length())									return E_POINTER;

	CRWThreadSemaphoreEngage _semaphore_engage(const_cast<CRWThreadSemaphore&>(m_lock), SEM_FALSE);

	if (!m_impl->m_scenario)
		return ERROR_SCENARIO_BAD_STATE;

	if ((m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_ASSET) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI90) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI95) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_FI100) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_RH) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_PRECIP) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_AREA) &&
		(m_impl->m_scenario->m_stepState != SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION_BURNDISTANCE))
		return ERROR_SCENARIO_BAD_STATE;

	AssetNode<fireengine_float_type>* node = m_impl->m_assetList.LH_Head();
	while (node->LN_Succ()) {
		if ((node->m_asset == asset) || ((asset == nullptr) && (m_impl->m_assetList.GetCount() == 1))) {
			AssetGeometryNode<fireengine_float_type>* g = nullptr;
			if (index != (std::uint32_t)-1) {
				g = node->m_geometry.IndexNode(index);
				if (!g)
					return ERROR_SCENARIO_ASSET_GEOMETRY_UNKNOWN;
			}
			return m_impl->m_scenario->ExportCriticalPath(node, g, flags, driver_name.c_str(), projection.c_str(), file_path.c_str(), *rules);
		}
		node = node->LN_Succ();
	}
	return ERROR_SCENARIO_ASSET_UNKNOWN;
}


std::int32_t CCWFGM_Scenario::serialVersionUid(const SerializeProtoOptions& options) const noexcept {
	if (options.fileVersion() == 2)
		return 6;
	return 5;
}


WISE::FireEngineProto::CwfgmScenario* CCWFGM_Scenario::serialize(const SerializeProtoOptions& options) {
	auto scenario = new WISE::FireEngineProto::CwfgmScenario();
	scenario->set_version(serialVersionUid(options));

	int fuelIndex = 0;
	_GUID guid;
	char name[10];
	while (!m_sp.GetPercentile(fuelIndex, &guid))
	{
		bool defaulted;
		if (SUCCEEDED(m_sp.IsPercentileDefault(&guid, &defaulted)) && (!defaulted || options.useVerboseOutput()))
		{
			memset(name, 0, 10);
			auto fuel = scenario->add_percentiles();
			double crown_s, surface_s;
			m_sp.GetPercentileValue(&guid, 0, &crown_s);
			m_sp.GetPercentileValue(&guid, 1, &surface_s);
			m_sp.GetPercentileName(&guid, name, 10);

			fuel->set_version(1);
			fuel->set_name(name);
			fuel->set_allocated_crown(DoubleBuilder().withValue(crown_s).forProtobuf(options.useVerboseFloats()));
			fuel->set_allocated_surface(DoubleBuilder().withValue(surface_s).forProtobuf(options.useVerboseFloats()));
		}
		fuelIndex++;
	}

	//FBP options
	{
		auto fbp = new WISE::FireEngineProto::CwfgmScenario_FbpOptions();

		fbp->set_allocated_terraineffect(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_TOPOGRAPHY)));
		fbp->set_allocated_windeffect(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_WIND)));

		scenario->set_allocated_fbpoptions(fbp);
	}
	//FGM options
	{
		auto fgm = new WISE::FireEngineProto::CwfgmScenario_FgmOptions();

		fgm->set_allocated_maxacceltimestep(HSS_Time::Serialization::TimeSerializer::serializeTimeSpan(m_temporalThreshold_Accel));
		fgm->set_allocated_distres(DoubleBuilder().withValue(m_spatialThreshold).forProtobuf(options.useVerboseFloats()));
		fgm->set_allocated_perimres(DoubleBuilder().withValue(m_perimeterResolution).forProtobuf(options.useVerboseFloats()));
		fgm->set_allocated_perimspacing(DoubleBuilder().withValue(m_perimeterSpacing).forProtobuf(options.useVerboseFloats()));
		fgm->set_allocated_minspreadros(DoubleBuilder().withValue(m_minimumROS).forProtobuf(options.useVerboseFloats()));
		fgm->set_allocated_stopatgridend(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_BOUNDARY_STOP)));
		fgm->set_allocated_breaching(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_BREACHING)));
		fgm->set_allocated_dynamicspatialthreshold(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC)));
		fgm->set_allocated_spotting(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_SPOTTING)));
		fgm->set_allocated_purgenondisplayable(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE)));
		if (m_dx != 0.0)
			fgm->set_allocated_dx(DoubleBuilder().withValue(m_dx).forProtobuf(options.useVerboseFloats()));
		if (m_dy != 0.0)
			fgm->set_allocated_dy(DoubleBuilder().withValue(m_dy).forProtobuf(options.useVerboseFloats()));
		if (m_dt != WTimeSpan(0ULL))
			fgm->set_allocated_dt(HSS_Time::Serialization::TimeSerializer::serializeTimeSpan(m_dt));
		if (m_dwd != 0.0)
			fgm->set_allocated_dwd(DoubleBuilder().withValue(m_dwd).forProtobuf(options.useVerboseFloats()));
		if (m_owd != -1.0)
			fgm->set_allocated_owd(DoubleBuilder().withValue(m_owd).forProtobuf(options.useVerboseFloats()));
		if (m_dvd != 0.0)
			fgm->set_allocated_dwd(DoubleBuilder().withValue(m_dvd).forProtobuf(options.useVerboseFloats()));
		if (m_ovd != -1.0)
			fgm->set_allocated_owd(DoubleBuilder().withValue(m_ovd).forProtobuf(options.useVerboseFloats()));
		fgm->set_allocated_growthpercentileapplied(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE_ENABLE)));
		if (m_growthPercentile >= 0.0)
			fgm->set_allocated_growthpercentile(DoubleBuilder().withValue(m_growthPercentile).forProtobuf(options.useVerboseFloats()));
        fgm->set_allocated_initialvertexcount(createProtobufObject((std::uint32_t)m_initialVertexCount));
        fgm->set_allocated_ignitionsize(DoubleBuilder().withValue(m_ignitionSize).forProtobuf(options.useVerboseFloats()));
		fgm->set_allocated_suppresstightconcaveadd(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT)));
		fgm->set_allocated_enablefalseorigin(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_FALSE_ORIGIN)));
		fgm->set_allocated_enablefalsescaling(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_FALSE_SCALING)));
		fgm->set_allocated_usecardinal(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_CARDINAL_ROS)));
		fgm->set_allocated_independenttimesteps(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS)));

		scenario->set_allocated_fgmoptions(fgm);
	}
	//FMC options
	{
		auto fmc = new WISE::FireEngineProto::CwfgmScenario_FmcOptions();

		if (m_optionFlags & (1ull << (CWFGM_SCENARIO_OPTION_SPECIFIED_FMC_ACTIVE)))
			fmc->set_allocated_peroverride(DoubleBuilder().withValue(m_specifiedFMC).forProtobuf(options.useVerboseFloats()));
		fmc->set_allocated_nodataelev(DoubleBuilder().withValue(m_defaultElevation).forProtobuf(options.useVerboseFloats()));
		fmc->set_allocated_terrain(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_FMC_TERRAIN)));
		fmc->set_allocated_accuratelocation(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION)));

		scenario->set_allocated_fmcoptions(fmc);
	}
	//FWI options
	{
		auto fwi = new WISE::FireEngineProto::CwfgmScenario_FwiOptions();

		fwi->set_allocated_fwispacialinterp(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL)));
		fwi->set_allocated_fwifromspacialweather(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI)));
		fwi->set_allocated_historyoneffectedfwi(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY)));
		fwi->set_allocated_fwitemporalinterp(createProtobufObject(TRUTH_FLAG(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMPORAL)));

		scenario->set_allocated_fwioptions(fwi);
	}

	if (m_sc.anythingValid())	// if some condition has been set
	{
		auto stop = new WISE::FireEngineProto::CwfgmScenario_StopModellingOptions();

		if (m_sc.responseTime.GetTotalSeconds() > 0) {
			stop->set_allocated_responsetime(HSS_Time::Serialization::TimeSerializer().serializeTimeSpan(m_sc.responseTime));
		}

		if (m_sc.fi100) {
			auto fi = new WISE::FireEngineProto::CwfgmScenario_StopModellingOptions_Threshold();
			fi->set_allocated_threshold(DoubleBuilder().withValue(m_sc.fi100PercentThreshold).forProtobuf(options.useVerboseFloats()));
			fi->set_allocated_duration(HSS_Time::Serialization::TimeSerializer().serializeTimeSpan(m_sc.fi100PercentDuration));
			stop->set_allocated_fi100percent(fi);
		}

		if (m_sc.fi95) {
			auto fi = new WISE::FireEngineProto::CwfgmScenario_StopModellingOptions_Threshold();
			fi->set_allocated_threshold(DoubleBuilder().withValue(m_sc.fi95PercentThreshold).forProtobuf(options.useVerboseFloats()));
			fi->set_allocated_duration(HSS_Time::Serialization::TimeSerializer().serializeTimeSpan(m_sc.fi95PercentDuration));
			stop->set_allocated_fi95percent(fi);
		}

		if (m_sc.fi90) {
			auto fi = new WISE::FireEngineProto::CwfgmScenario_StopModellingOptions_Threshold();
			fi->set_allocated_threshold(DoubleBuilder().withValue(m_sc.fi90PercentThreshold).forProtobuf(options.useVerboseFloats()));
			fi->set_allocated_duration(HSS_Time::Serialization::TimeSerializer().serializeTimeSpan(m_sc.fi90PercentDuration));
			stop->set_allocated_fi90percent(fi);
		}

		if (m_sc.precip) {
			auto fi = new WISE::FireEngineProto::CwfgmScenario_StopModellingOptions_Threshold();
			fi->set_allocated_threshold(DoubleBuilder().withValue(m_sc.PrecipThreshold).forProtobuf(options.useVerboseFloats()));
			fi->set_allocated_duration(HSS_Time::Serialization::TimeSerializer().serializeTimeSpan(m_sc.PrecipDuration));
			stop->set_allocated_precip(fi);
		}

		if (m_sc.RH) {
			auto fi = new WISE::FireEngineProto::CwfgmScenario_StopModellingOptions_Threshold();
			fi->set_allocated_threshold(DoubleBuilder().withValue(m_sc.RHThreshold * 100.0).forProtobuf(options.useVerboseFloats()));
			fi->set_allocated_duration(HSS_Time::Serialization::TimeSerializer().serializeTimeSpan(m_sc.RHDuration));
			stop->set_allocated_rh(fi);
		}

		if (m_sc.area) {
			stop->set_allocated_area(DoubleBuilder().withValue(m_sc.areaThreshold).forProtobuf(options.useVerboseFloats()));
		}

		if (m_sc.burnDistance) {
			stop->set_allocated_burndistance(DoubleBuilder().withValue(m_sc.burnDistanceThreshold).forProtobuf(options.useVerboseFloats()));
		}

		scenario->set_allocated_stopoptions(stop);
	}

	scenario->set_allocated_starttime(HSS_Time::Serialization::TimeSerializer().serializeTime(m_startTime, options.fileVersion()));
	scenario->set_allocated_endtime(HSS_Time::Serialization::TimeSerializer().serializeTime(m_endTime, options.fileVersion()));
	scenario->set_allocated_displayinterval(HSS_Time::Serialization::TimeSerializer().serializeTimeSpan(m_displayInterval));

	return scenario;
}


std::string append_msg(std::string& existing, const std::string &append) {
	std::string new_msg;
	if (!existing.empty()) {
		new_msg = existing;
		new_msg += "\n";
		new_msg += append;
		return new_msg;
	}
	else
		return append;
}


CCWFGM_Scenario *CCWFGM_Scenario::deserialize(const google::protobuf::Message& proto, std::shared_ptr<validation::validation_object> valid, const std::string& name)
{
	auto scenario = dynamic_cast_assert<const WISE::FireEngineProto::CwfgmScenario*>(&proto);
	double dValue;
	uint16_t sValue;
	WTimeSpan tsValue;

	if (!scenario)
	{
		if (valid)
			/// <summary>
			/// The object passed as a scenario is invalid. An incorrect object type was passed to the parser.
			/// </summary>
			/// <type>internal</type>
			valid->add_child_validation("WISE.FireEngineProto.CwfgmScenario", name, validation::error_level::SEVERE, validation::id::object_invalid, proto.GetDescriptor()->name());
		weak_assert(0);
		throw ISerializeProto::DeserializeError("WISE.FireEngineProto.CwfgmScenario: Protobuf object invalid", ERROR_PROTOBUF_OBJECT_INVALID);
	}
	if ((scenario->version() < 1) || (scenario->version() > 6))
	{
		if (valid)
			/// <summary>
			/// The object version is not supported. The scenario is not supported by this version of Prometheus.
			/// </summary>
			/// <type>user</type>
			valid->add_child_validation("WISE.FireEngineProto.CwfgmScenario", name, validation::error_level::SEVERE, validation::id::version_mismatch, std::to_string(scenario->version()));
		weak_assert(0);
		throw ISerializeProto::DeserializeError("WISE.FireEngineProto.CwfgmScenario: Version is invalid", ERROR_PROTOBUF_OBJECT_VERSION_INVALID);
	}
	if (scenario->version() < 6) {
		if (valid)
			/// <summary>
			/// The object version is out of date, but still supported by this version of Prometheus.
			/// </summary>
			/// <type>user</type>
			valid->add_child_validation("WISE.FireEngineProto.CwfgmScenario", name, validation::error_level::INFORMATION, validation::id::version_not_current, std::to_string(scenario->version()));
	}

	std::string throw_msg;
	auto vt = validation::conditional_make_object(valid, "WISE.FireEngineProto.CwfgmScenario", name);
	auto v = vt.lock();

	if (scenario->has_starttime())
	{
		auto time = HSS_Time::Serialization::TimeSerializer().deserializeTime(scenario->starttime(), m_timeManager, valid, "Scenario.StartTime");
		m_startTime = WTime(*time);
		delete time;
	}
	if ((m_startTime < WTime::GlobalMin(m_timeManager)) || (m_startTime > WTime::GlobalMax(m_timeManager)))
	{
		m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid scenario start time";
		if (v)
			/// <summary>
			/// The scenario start time is out of range or invalid.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTime", "startTime", validation::error_level::SEVERE, validation::id::time_invalid, m_startTime.ToString(WTIME_FORMAT_STRING_ISO8601), { true, WTime::GlobalMin().ToString(WTIME_FORMAT_STRING_ISO8601) }, { true, WTime::GlobalMax().ToString(WTIME_FORMAT_STRING_ISO8601) });
		throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid scenario start time");
	}
	if (m_startTime.GetMicroSeconds(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST)) {
		m_loadWarning += "Warning: fractions of seconds on the start time will be purged to the start of the second.";
		if (v)
			/// <summary>
			/// The scenario start time contains fractions of seconds.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTime", "startTime", validation::error_level::WARNING, validation::id::time_invalid, m_startTime.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged.");
		m_startTime.PurgeToSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	}

	if (scenario->has_endtime())
	{
		auto time = HSS_Time::Serialization::TimeSerializer().deserializeTime(scenario->endtime(), m_timeManager, valid, "Scenario.EndTime");
		m_endTime = WTime(*time);
		delete time;
	}
	if ((m_endTime < WTime::GlobalMin(m_timeManager)) || (m_endTime > WTime::GlobalMax(m_timeManager)))
	{
		m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid scenario end time";
		if (v)
			/// <summary>
			/// The scenario end time is out of range or invalid.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTime", "endTime", validation::error_level::SEVERE, validation::id::time_invalid, m_endTime.ToString(WTIME_FORMAT_STRING_ISO8601), { true, WTime::GlobalMin().ToString(WTIME_FORMAT_STRING_ISO8601) }, { true, WTime::GlobalMax().ToString(WTIME_FORMAT_STRING_ISO8601) });
		throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid scenario end time");
	}
	if (m_endTime.GetMicroSeconds(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST)) {
		m_loadWarning += "Warning: fractions of seconds on the end time will be purged to the start of the second.";
		if (v)
			/// <summary>
			/// The end time contains fractions of seconds.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTime", "endTime", validation::error_level::WARNING, validation::id::time_invalid, m_endTime.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged.");
		m_endTime.PurgeToSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	}

	if ((m_startTime > m_endTime) || ((m_startTime + WTimeSpan(365, 0, 0, 0)) < m_endTime)) {
		if (v) {
			if (m_startTime > m_endTime) {
				/// <summary>
				/// The scenario start time occurs after the end time.
				/// </summary>
				/// <type>user</type>
				v->add_child_validation("HSS.Times.WTime", { "startTime", "endTime" }, validation::error_level::SEVERE, validation::id::time_invalid, { m_startTime.ToString(WTIME_FORMAT_STRING_ISO8601), m_endTime.ToString(WTIME_FORMAT_STRING_ISO8601) });
			}
			else /*if (m_startTime + WTimeSpan(365, 0, 0, 0)) < m_endTime)*/ {
				/// <summary>
				/// The duration of the scenario exceeds one year.
				/// </summary>
				/// <type>user</type>
				v->add_child_validation("HSS.Times.WTime", { "endTime", "startTime" }, validation::error_level::SEVERE, validation::id::time_range_invalid, { m_endTime.ToString(WTIME_FORMAT_STRING_ISO8601), (m_startTime + WTimeSpan(365, 0, 0, 0)).ToString(WTIME_FORMAT_STRING_ISO8601) });
			}
		}
		throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid scenario times");
	}

	if (scenario->has_fbpoptions())
	{
		auto fbp = scenario->fbpoptions();
		if (fbp.has_terraineffect())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_TOPOGRAPHY, fbp.terraineffect().value());
		if (fbp.has_windeffect())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_WIND, fbp.windeffect().value());
	}
	if (scenario->has_fgmoptions())
	{
		auto vt2 = validation::conditional_make_object(v, "WISE.FireEngineProto.CwfgmScenario.FgmOptions", "fgmOptions");
		auto v2 = vt2.lock();

		auto fgm = scenario->fgmoptions();
		if (fgm.has_stopatgridend())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_BOUNDARY_STOP, fgm.stopatgridend().value());
		if (fgm.has_breaching())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_BREACHING, fgm.breaching().value());
		if (fgm.has_dynamicspatialthreshold())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC, fgm.dynamicspatialthreshold().value());
		if (fgm.has_spotting())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_SPOTTING, fgm.spotting().value());
		if (fgm.has_distres()) {
			dValue = DoubleBuilder().withProtobuf(fgm.distres()).getValue();

			if (dValue < 0.2) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid spatial threshold value";
				if (v2)
					/// <summary>
					/// The scenario's distance resolution must be between 0.2 and 10.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "distRes", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.2 }, { true, 10.0 });
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid spatial threshold value");
			}
			if (dValue > 10.0) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid spatial threshold value";
				if (v2)
					/// <summary>
					/// The scenario's distance resolution must be between 0.2 and 10.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "distRes", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.2 }, { true, 10.0 });
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid spatial threshold value");
			}

			m_spatialThreshold = dValue;
		}

		if (fgm.has_perimres()) {
			dValue = DoubleBuilder().withProtobuf(fgm.perimres()).getValue();

			if (dValue < 0.2) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid perimeter resolution value";
				if (v2)
					/// <summary>
					/// The scenario's perimeter resolution must be between 0.2 and 10.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "perimRes", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.2 }, { true, 10.0 });
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid perimeter resolution value");
			}
			if (dValue > 10.0) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid perimeter resolution value";
				if (v2)
					/// <summary>
					/// The scenario's perimeter resolution must be between 0.2 and 10.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "perimRes", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.2 }, { true, 10.0 });
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid perimeter resolution value");
			}

			m_perimeterResolution = dValue;
		}
		if (fgm.has_perimspacing())
		{
			dValue = fgm.perimspacing().value();

			if (dValue < 0.0) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid minimum perimeter spacing value";
				if (v2)
					/// <summary>
					/// The scenario's perimeter minimum perimeter spacing must be between 0.0 and 10.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "perimSpacing", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 10.0 });
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid minimum perimeter spacing value");
			}
			if (dValue > 10.0) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid minimum perimeter spacing value";
				if (v2)
					/// <summary>
					/// The scenario's perimeter minimum perimeter spacing must be between 0.0 and 10.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "perimSpacing", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 10.0 });
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid minimum perimeter spacing value");
			}

			m_perimeterSpacing = dValue;
		}
		if (fgm.has_minspreadros()) {
			dValue = DoubleBuilder().withProtobuf(fgm.minspreadros()).getValue();

			if (dValue < 0.0000001) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid minimum ROS value";
				if (v2)
					/// <summary>
					/// The scenario's minimum spreading ROS must be between 0.0000001 and 1.0m, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "minSpreadRos", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0000001 }, { true, 1.0 }, "m");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid minimum ROS value");
			}
			if (dValue > 1.0) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid minimum ROS value";
				if (v2)
					/// <summary>
					/// The scenario's minimum spreading ROS must be between 0.0000001 and 1.0m, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "minSpreadRos", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0000001 }, { true, 1.0 }, "m");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid minimum ROS value");
			}

			m_minimumROS = dValue;
		}
		if (fgm.has_maxacceltimestep())
		{
			auto span = HSS_Time::Serialization::TimeSerializer::deserializeTimeSpan(fgm.maxacceltimestep(), valid, "Scenario.MaxAccelTimeStep");
			tsValue = WTimeSpan(*span);

			if ((tsValue.GetTotalSeconds() < 0) && (tsValue.GetTotalSeconds() != -1)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid temporal threshold accel value";
				if (v2)
					/// <summary>
					/// The scenario's maximum acceleration time step must be positive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "maxAccelTimestep", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), "Maximum acceleration duration cannot be negative.  Use -1 to displable acceleration-specific timestep maximum durations.");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid temporal threshold accel value");
			}
			if (tsValue.GetMicroSeconds()) {
				m_loadWarning += "Warning: fractions of seconds on the acceleration time span will be purged to the start of the second.";
				if (v)
					/// <summary>
					/// The scenario's maximum acceleration time step must not have fractions of seconds.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "maxAccelTimestep", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged");
				tsValue.PurgeToSecond();
			}

			m_temporalThreshold_Accel = tsValue;
			delete span;
		}
		if (fgm.has_purgenondisplayable())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE, fgm.purgenondisplayable().value());
		if (fgm.has_dx()) {
			dValue = DoubleBuilder().withProtobuf(fgm.dx()).getValue();

			if ((dValue < -250.0) || (dValue > 250.0)) { 
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid dx value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dx value must be <= +-250m, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "dx", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, -250.0 }, { true, 250.0 }, "m");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid dx value");
			}

			m_dx = dValue;
		}
		if (fgm.has_dy()) {
			dValue = DoubleBuilder().withProtobuf(fgm.dy()).getValue();

			if ((dValue < -250.0) || (dValue > 250.0)) {
				m_loadWarning = "CWFGM_Scenario: Invalid dy value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dy value must be <= +-250m, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "dy", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, -250.0 }, { true, 250.0 }, "m");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid dy value");
			}

			m_dy = dValue;
		}
		if (fgm.has_dt())
		{
			auto span = HSS_Time::Serialization::TimeSerializer::deserializeTimeSpan(fgm.dt(), valid, "Scenario.dT");
			tsValue = WTimeSpan(*span);

			if ((tsValue.GetTotalSeconds()) > (4 * 60 * 60))
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid dt value";
				if (v2)
					/// <summary>
					/// The scenario's dt value must be <= +-4 hours, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "dt", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, WTimeSpan(0, -4, 0, 0).ToString(WTIME_FORMAT_TIME) }, { true, WTimeSpan(0, 4, 0, 0).ToString(WTIME_FORMAT_TIME) });
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid dt value");
			}
			if ((tsValue.GetTotalSeconds()) < (-4 * 60 * 60))
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid dt value";
				if (v2)
					/// <summary>
					/// The scenario's dt value must be <= +-4 hours, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "dt", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, WTimeSpan(0, -4, 0, 0).ToString(WTIME_FORMAT_TIME) }, { true, WTimeSpan(0, 4, 0, 0).ToString(WTIME_FORMAT_TIME) });
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid dt value");
			}

			m_dt = tsValue;
			delete span;
		}
		if (fgm.has_dwd())
		{
			dValue = DoubleBuilder().withProtobuf(fgm.dwd()).getValue();

			if ((dValue < -360.0) || (dValue > 360.0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid dWD value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dwd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "dWD", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, -360.0 }, { true, 360.0 }, "degrees");
				throw_msg = append_msg(throw_msg, "Error: WISE.FireEngineProto.CwfgmScenario: Invalid dWD value");
			}

			m_dwd = dValue;
		}
		if (fgm.has_owd())
		{
			dValue = DoubleBuilder().withProtobuf(fgm.owd()).getValue();

			if (((dValue != -1.0) && (dValue < 0.0)) || (dValue > 360.0))
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid oWD value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's owd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "oWD", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 360.0 }, "degrees");
				throw_msg = append_msg(throw_msg, "Error: WISE.FireEngineProto.CwfgmScenario: Invalid dWD value");
			}

			m_owd = dValue;
		}
		if (fgm.has_dvd())
		{
			dValue = DoubleBuilder().withProtobuf(fgm.dvd()).getValue();

			if ((dValue < -360.0) || (dValue > 360.0))
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid dWD value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "dVD", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, -360.0 }, { true, 360.0 }, "degrees");
				throw_msg = append_msg(throw_msg, "Error: WISE.FireEngineProto.CwfgmScenario: Invalid dVD value");
			}

			m_dvd = dValue;
		}
		if (fgm.has_ovd())
		{
			dValue = DoubleBuilder().withProtobuf(fgm.ovd()).getValue();

			if (((dValue != -1.0) && (dValue < 0.0)) || (dValue > 360.0))
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid oVD value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's ovd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "oVD", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 360.0 }, "degrees");
				throw_msg = append_msg(throw_msg, "Error: WISE.FireEngineProto.CwfgmScenario: Invalid oVD value");
			}

			m_ovd = dValue;
		}
		if (fgm.has_growthpercentileapplied())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE_ENABLE, fgm.growthpercentileapplied().value());
		if (fgm.has_growthpercentile())
		{
			dValue = DoubleBuilder().withProtobuf(fgm.growthpercentile()).getValue();

			if ((dValue <= 0.0) || (dValue >= 100.0))
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid growth percentile value";
				if (v2)
					/// <summary>
					/// The scenario's percentile value must be between 0 and 100, exclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "growthPercentile", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { false, 0.0 }, { false, 100.0 }, "%%");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid growth percentile value");
			}

			m_growthPercentile = dValue;
		}
        if (fgm.has_initialvertexcount())
        {
			sValue = fgm.initialvertexcount().value();

			if (sValue < 6)
			{
				m_loadWarning = "Warning: CWFGM_Scenario: Invalid initial vertex count";
				if (v2)
					/// <summary>
					/// The scenario's initial vertex count must be between 6 and 64, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "initialVertexCount", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(sValue), { true, 6 }, { true, 64 });
				sValue = 6;
			}
			else if (m_initialVertexCount > 64)
			{
				m_loadWarning = "Warning: CWFGM_Scenario: Invalid initial vertex count";
				if (v2)
					/// <summary>
					/// The scenario's initial vertex count must be between 6 and 64, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "initialVertexCount", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(sValue), { true, 6 }, { true, 64 });
				sValue = 64;
			}

			m_initialVertexCount = sValue;
        }
        if (fgm.has_ignitionsize())
        {
			dValue = DoubleBuilder().withProtobuf(fgm.ignitionsize()).getValue();

			if (dValue <= 0.0)
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid ignition size value";
				if (v2)
					/// <summary>
					/// The scenario's initial ignition size must be <= 25.0m.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "ignitionSize", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 25.0 }, "m");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid ignition size value");
			}
			if (dValue > 25.0)
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid ignition size value";
				if (v2)
					/// <summary>
					/// The scenario's initial ignition size must be <= 25.0m.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "ignitionSize", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 25.0 }, "m");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid ignition size value");
			}

			m_ignitionSize = dValue;
        }

		if (fgm.has_suppresstightconcaveadd())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT, fgm.suppresstightconcaveadd().value());
		if (fgm.has_enablefalseorigin())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_FALSE_ORIGIN, fgm.enablefalseorigin().value());
		if (fgm.has_enablefalsescaling())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_FALSE_SCALING, fgm.enablefalsescaling().value());

#ifndef USE_BIGFLOATS
		m_optionFlags |= (1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN) | (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING);
#else
		m_optionFlags &= (~((1ull << CWFGM_SCENARIO_OPTION_FALSE_ORIGIN) | (1ull << CWFGM_SCENARIO_OPTION_FALSE_SCALING)));
#endif			// if we aren't using 128-bit floats, then we are using 64-bit floats and we know we have issues in UTM space, so force this on

		if (fgm.has_usecardinal())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_CARDINAL_ROS, fgm.usecardinal().value());
		if (fgm.has_independenttimesteps())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_INDEPENDENT_TIMESTEPS, fgm.independenttimesteps().value());
	}

	if (scenario->has_fmcoptions())
	{
		auto vt2 = validation::conditional_make_object(v, "WISE.FireEngineProto.CwfgmScenario.FmcOptions", "fmcOptions");
		auto v2 = vt2.lock();

		auto fmc = scenario->fmcoptions();
		if (fmc.has_nodataelev()) {
			dValue = DoubleBuilder().withProtobuf(fmc.nodataelev()).getValue();

			if (dValue < 0.0) if ((dValue != -99.0) && (dValue != -1.0))
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid default elevation value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's elevation value for NODATA value is invalid.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "nodataElev", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 7000.0 }, "m", "-1.0, -99 are exceptions, refer to documentation.");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid default elevation value");
			}
			if (dValue > 7000.0)
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid default elevation value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's elevation value for NODATA value is invalid.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "nodataElev", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 7000.0 }, "m", "-1.0, -99 are exceptions, refer to documentation.");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid default elevation value");
			}

			m_defaultElevation = dValue;
		}
		if (fmc.has_terrain())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_FMC_TERRAIN, fmc.terrain().value());
		if (fmc.has_accuratelocation())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION, fmc.accuratelocation().value());
		if (fmc.has_peroverride())
		{
			dValue = DoubleBuilder().withProtobuf(fmc.peroverride()).getValue();

			if ((dValue < 0.0) && (dValue != -1.0))
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid specified FMC value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's override FMC value is invalid.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "perOverride", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 300.0 }, "%%", "-1.0 = no value specified");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid specified FMC value");
			}
			else if (dValue == -1.0) {
				m_loadWarning = "Warning: CWFGM_Scenario: Specifying a FMC value of -1.0 is equivalent to not specifying an FMC value.";
			}
			else if (dValue > 300.0)
			{
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid specified FMC value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's override FMC value is invalid.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "perOverride", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 300.0 }, "%%", "-1.0 = no value specified");
				throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid specified FMC value");
			}

			if (dValue != -1.0) {
				m_optionFlags |= (1ull << CWFGM_SCENARIO_OPTION_SPECIFIED_FMC_ACTIVE);
				m_specifiedFMC = dValue;
			}
		}
	}
	if (scenario->has_fwioptions())
	{
		auto fwi = scenario->fwioptions();
		if (fwi.has_fwifromspacialweather())
		{
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_PRECIP, fwi.fwifromspacialweather().value());
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMP_RH, fwi.fwifromspacialweather().value());
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI, fwi.fwifromspacialweather().value());
		}
		if (fwi.has_historyoneffectedfwi())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY, fwi.historyoneffectedfwi().value());
		if (fwi.has_fwispacialinterp())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL, fwi.fwispacialinterp().value());
		if (fwi.has_fwitemporalinterp())
			FLAG_TRUTH(m_optionFlags, CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMPORAL, fwi.fwitemporalinterp().value());
	}
	
	auto timespan = HSS_Time::Serialization::TimeSerializer().deserializeTimeSpan(scenario->displayinterval(), valid, "displayInterval");
	tsValue = WTimeSpan(*timespan);

	if ((((LONGLONG)tsValue.GetTotalSeconds()) < 0) && (tsValue.GetTotalSeconds() != (ULONGLONG)-1))
	{
		m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario: Invalid display timestep";
		if (v)
			/// <summary>
			/// The scenario's display interval must be positive.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("Math.Double", "displayInterval", validation::error_level::SEVERE, validation::id::value_invalid, std::to_string(dValue), "displayInterval duration cannot be negative");
		throw_msg = append_msg(throw_msg, "WISE.FireEngineProto.CwfgmScenario: Invalid specified display interval");
	}
	if (tsValue.GetMicroSeconds()) {
		m_loadWarning += "Warning: fractions of seconds on the display time step will be purged to the start of the second.";
		if (v)
			/// <summary>
			/// The scenario's display interval must not have fractions of seconds.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTimeSpan", "displayInterval", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged");
		tsValue.PurgeToSecond();
	}

	m_displayInterval = tsValue;
	delete timespan;

	if (m_defaultElevation < 0.0 && m_defaultElevation != -99.0)
		m_defaultElevation = -1.0;

	{
		_GUID guid;
		for (int i = 0; i < scenario->percentiles_size(); i++)
		{
			auto vt2 = validation::conditional_make_object(v, "WISE.FireEngineProto.CwfgmScenario.ScenarioFuelPercentile", strprintf("percentiles[%d]", i));
			auto v2 = vt2.lock();
			auto fuel = scenario->percentiles(i);
			if (SUCCEEDED(m_sp.GetPercentile(fuel.name().c_str(), &guid)))
			{

				double p = DoubleBuilder().withProtobuf(fuel.crown()).getValue();
				if ((p > 0.0) && (p < 100.0))
					m_sp.SetPercentileValue(&guid, 0, p);
				else {
					if (v2)
						/// <summary>
						/// The scenario's specific fueltype percentile value must be between 0 and 100, exclusive.
						/// </summary>
						/// <type>user</type>
						v2->add_child_validation("Math.Double", "crown", validation::error_level::WARNING, validation::id::value_invalid, fuel.name(), { false, 0.0 }, { false, 100.0 });
				}
				p = DoubleBuilder().withProtobuf(fuel.surface()).getValue();
				if ((p > 0.0) && (p < 100.0))
					m_sp.SetPercentileValue(&guid, 1, p);
				else {
					if (v2)
						/// <summary>
						/// The scenario's specific fueltype percentile value must be between 0 and 100, exclusive.
						/// </summary>
						/// <type>user</type>
						v2->add_child_validation("Math.Double", "surface", validation::error_level::WARNING, validation::id::value_invalid, fuel.name(), { false, 0.0 }, { false, 100.0 });
				}
			}
			else {
				if (v2)
					/// <summary>
					/// A percentile value was specified for an unknown fueltype.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("string", "name", validation::error_level::WARNING, validation::id::value_invalid, fuel.name(), "Percentile fueltype name is unknown");
			}
		}
	}

	if (scenario->has_stopoptions()) {
		auto vt2 = validation::conditional_make_object(v, "WISE.FireEngineProto.CwfgmScenario.StopModellingOptions", "stopOptions");
		auto v2 = vt2.lock();

		if (scenario->stopoptions().has_responsetime()) {
			auto timespan = HSS_Time::Serialization::TimeSerializer().deserializeTimeSpan(scenario->stopoptions().responsetime(), valid, "Response Time");
			m_sc.responseTime = WTimeSpan(*timespan);

			if (m_sc.responseTime < WTimeSpan(0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.responseTime: Invalid value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The response time must be >= 0.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "responseTime", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Value must be greater than or equal to 0");
				m_sc.responseTime = WTimeSpan(0, 0, 0, 0);

			}

			if (m_sc.responseTime.GetMicroSeconds()) {
				m_loadWarning += "Warning: fractions of seconds on the duration will be purged to the start of the second.";
				if (v)
					/// <summary>
					/// The response time must not have fractions of seconds.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "fi100PercentThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged");
				m_sc.responseTime.PurgeToSecond();
			}
		}

		if (scenario->stopoptions().has_fi100percent()) {
			m_sc.fi100 = true;
			m_sc.fi100PercentThreshold = DoubleBuilder().withProtobuf(scenario->stopoptions().fi100percent().threshold()).getValue();
			auto timespan = HSS_Time::Serialization::TimeSerializer().deserializeTimeSpan(scenario->stopoptions().fi100percent().duration(), valid, "100% FI threshold duration");
			m_sc.fi100PercentDuration = WTimeSpan(*timespan);

			if ((m_sc.fi100PercentThreshold < 0.0) || (m_sc.fi100PercentThreshold > 20000.0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.fi100Percent: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "fi100Percent", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 20000.0 }, "kW/m");
				if (m_sc.fi100PercentThreshold < 0.0)
					m_sc.fi100PercentThreshold = 0.0;
				if (m_sc.fi100PercentThreshold > 20000.0)
					m_sc.fi100PercentThreshold = 20000.0;
			}

			if (m_sc.fi100PercentDuration < WTimeSpan(0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.fi100Percent: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "fi100PercentThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Value must be greater than 0");
				m_sc.fi100PercentDuration = WTimeSpan(0, 0, 0, 1);

			}

			if (m_sc.fi100PercentDuration.GetMicroSeconds()) {
				m_loadWarning += "Warning: fractions of seconds on the duration will be purged to the start of the second.";
				if (v)
					/// <summary>
					/// The scenario's display interval must not have fractions of seconds.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "fi100PercentThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged");
				m_sc.fi100PercentDuration.PurgeToSecond();
			}
		}

		if (scenario->stopoptions().has_fi95percent()) {
			m_sc.fi95 = true;
			m_sc.fi95PercentThreshold = DoubleBuilder().withProtobuf(scenario->stopoptions().fi95percent().threshold()).getValue();
			auto timespan = HSS_Time::Serialization::TimeSerializer().deserializeTimeSpan(scenario->stopoptions().fi95percent().duration(), valid, "95% FI threshold duration");
			m_sc.fi95PercentDuration = WTimeSpan(*timespan);

			if ((m_sc.fi95PercentThreshold < 0.0) || (m_sc.fi95PercentThreshold > 20000.0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.fi95Percent: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "fi95Percent", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 20000.0 }, "kW/m");
				if (m_sc.fi95PercentThreshold < 0.0)
					m_sc.fi95PercentThreshold = 0.0;
				if (m_sc.fi95PercentThreshold > 20000.0)
					m_sc.fi95PercentThreshold = 20000.0;
			}

			if (m_sc.fi95PercentDuration <= WTimeSpan(0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.fi95Percent: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "fi95PercentThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Value must be greater than 0");
				m_sc.fi95PercentDuration = WTimeSpan(0, 0, 0, 1);

			}

			if (m_sc.fi95PercentDuration.GetMicroSeconds()) {
				m_loadWarning += "Warning: fractions of seconds on the duration will be purged to the start of the second.";
				if (v)
					/// <summary>
					/// The scenario's display interval must not have fractions of seconds.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "fi95PercentThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged");
				m_sc.fi95PercentDuration.PurgeToSecond();
			}
		}

		if (scenario->stopoptions().has_fi90percent()) {
			m_sc.fi90 = true;
			m_sc.fi90PercentThreshold = DoubleBuilder().withProtobuf(scenario->stopoptions().fi90percent().threshold()).getValue();
			auto timespan = HSS_Time::Serialization::TimeSerializer().deserializeTimeSpan(scenario->stopoptions().fi90percent().duration(), valid, "90% FI threshold duration");
			m_sc.fi90PercentDuration = WTimeSpan(*timespan);

			if ((m_sc.fi90PercentThreshold < 0.0) || (m_sc.fi90PercentThreshold > 20000.0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.fi90Percent: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "fi90Percent", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 20000.0 }, "kW/m");
				if (m_sc.fi90PercentThreshold < 0.0)
					m_sc.fi90PercentThreshold = 0.0;
				if (m_sc.fi90PercentThreshold > 20000.0)
					m_sc.fi90PercentThreshold = 20000.0;
			}

			if (m_sc.fi90PercentDuration <= WTimeSpan(0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.fi90Percent: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "fi90PercentThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Value must be greater than 0");
				m_sc.fi90PercentDuration = WTimeSpan(0, 0, 0, 1);

			}

			if (m_sc.fi90PercentDuration.GetMicroSeconds()) {
				m_loadWarning += "Warning: fractions of seconds on the duration will be purged to the start of the second.";
				if (v)
					/// <summary>
					/// The scenario's display interval must not have fractions of seconds.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "fi90PercentThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged");
				m_sc.fi90PercentDuration.PurgeToSecond();
			}
		}

		if (scenario->stopoptions().has_precip()) {
			m_sc.precip = true;
			m_sc.PrecipThreshold = DoubleBuilder().withProtobuf(scenario->stopoptions().precip().threshold()).getValue();
			auto timespan = HSS_Time::Serialization::TimeSerializer().deserializeTimeSpan(scenario->stopoptions().precip().duration(), valid, "Precip threshold duration");
			m_sc.PrecipDuration = WTimeSpan(*timespan);

			if ((m_sc.PrecipThreshold < 0.0) || (m_sc.PrecipThreshold > 200.0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.Precip: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "Precip", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 200.0 }, "mm");
				if (m_sc.PrecipThreshold < 0.0)
					m_sc.PrecipThreshold = 0.0;
				if (m_sc.PrecipThreshold > 200.0)
					m_sc.PrecipThreshold = 200.0;
			}

			if (m_sc.PrecipDuration <= WTimeSpan(0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.Precip: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "PrecipDuration", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Value must be greater than 0");
				m_sc.PrecipDuration = WTimeSpan(0, 0, 0, 1);
			}

			if (m_sc.PrecipDuration.GetMicroSeconds()) {
				m_loadWarning += "Warning: fractions of seconds on the duration will be purged to the start of the second.";
				if (v)
					/// <summary>
					/// The scenario's display interval must not have fractions of seconds.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "PrecipThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged");
				m_sc.PrecipDuration.PurgeToSecond();
			}
		}

		if (scenario->stopoptions().has_rh()) {
			m_sc.RH = true;
			m_sc.RHThreshold = DoubleBuilder().withProtobuf(scenario->stopoptions().rh().threshold()).getValue();
			auto timespan = HSS_Time::Serialization::TimeSerializer().deserializeTimeSpan(scenario->stopoptions().rh().duration(), valid, "RH threshold duration");
			m_sc.RHDuration = WTimeSpan(*timespan);

			if ((m_sc.RHThreshold < 0.0) || (m_sc.RHThreshold > 100.0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.RH: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "RH", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(dValue), { true, 0.0 }, { true, 100.0 }, "%%");
				if (m_sc.RHThreshold < 0.0)
					m_sc.RHThreshold = 0.0;
				if (m_sc.RHThreshold > 200.0)
					m_sc.RHThreshold = 200.0;
			}
			m_sc.RHThreshold *= 0.01; // convert from percentage to decimal

			if (m_sc.RHDuration <= WTimeSpan(0)) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.RH: Invalid threshold value";
				weak_assert(0);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "RHThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Value must be greater than 0");
				m_sc.RHDuration = WTimeSpan(0, 0, 0, 1);
			}

			if (m_sc.RHDuration.GetMicroSeconds()) {
				m_loadWarning += "Warning: fractions of seconds on the duration will be purged to the start of the second.";
				if (v)
					/// <summary>
					/// The scenario's display interval must not have fractions of seconds.
					/// </summary>
					/// <type>user</type>
					v->add_child_validation("HSS.Times.WTimeSpan", "RHThreshold", validation::error_level::WARNING, validation::id::value_invalid, tsValue.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged");
				m_sc.RHDuration.PurgeToSecond();
			}
		}

		if (scenario->stopoptions().has_area()) {
			m_sc.area = true;
			m_sc.areaThreshold = scenario->stopoptions().area().value();

			if (m_sc.areaThreshold < 0.0) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.Area: Invalid threshold value";
				weak_assert(false);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "Area", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(m_sc.areaThreshold));
				m_sc.areaThreshold = 0.0;
				m_sc.area = false;
			}
		}

		if (scenario->stopoptions().has_burndistance()) {
			m_sc.burnDistance = true;
			m_sc.burnDistanceThreshold = scenario->stopoptions().burndistance().value();

			if (m_sc.burnDistanceThreshold < 0.0) {
				m_loadWarning = "Error: WISE.FireEngineProto.CwfgmScenario.StopModellingOptions.BurnDistance: Invalid threshold value";
				weak_assert(false);
				if (v2)
					/// <summary>
					/// The scenario's dvd value must be <= +-360.0, inclusive.
					/// </summary>
					/// <type>user</type>
					v2->add_child_validation("Math.Double", "Area", validation::error_level::WARNING, validation::id::value_invalid, std::to_string(m_sc.areaThreshold));
				m_sc.burnDistanceThreshold = 0.0;
				m_sc.burnDistance = false;
			}
		}
	}

	weak_assert(m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_FMC_TERRAIN));
	m_optionFlags &= (~(1ull << CWFGM_SCENARIO_OPTION_USE_2DGROWTH));	// now only using 3d equations
	weak_assert(m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION));
	m_optionFlags |= (1ull << CWFGM_SCENARIO_OPTION_FMC_TERRAIN) | (1ull << CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION);

	if (scenario->version() < 2) {
		if (m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT)) {
			m_loadWarning += "Scenario: Turned off the choice to suppress introducing points in concave portions of the fire perimeter.\n";
			m_optionFlags &= (~(1ull << CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT));
		}
		if (m_perimeterSpacing == 0.0) {
			m_loadWarning += "Scenario: Setting minimum perimeter spacing to 1mm.\n";
			m_perimeterSpacing = 0.001;
		}
	}
	if (scenario->version() < 5) {
#define CWFGM_SCENARIO_OPTION_NONFUELS_AS_VECTOR_BREAKS	15
#define CWFGM_SCENARIO_OPTION_BACKFILL_NONFUELS_TO_VECTOR_BREAKS 32

		if (m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_NONFUELS_AS_VECTOR_BREAKS)) {
			m_loadWarning += "Scenario: deprecating building vector breaks around non-fuels.\n";
			m_optionFlags &= (~(1ull << CWFGM_SCENARIO_OPTION_NONFUELS_AS_VECTOR_BREAKS));
		}
		if (m_optionFlags & (1ull << CWFGM_SCENARIO_OPTION_BACKFILL_NONFUELS_TO_VECTOR_BREAKS)) {
			m_loadWarning += "Scenario: deprecating building vector breaks around non-fuel grid cells that were missed or dropped by the untangler.\n";
			m_optionFlags &= (~(1ull << CWFGM_SCENARIO_OPTION_BACKFILL_NONFUELS_TO_VECTOR_BREAKS));
		}
	}

	if (throw_msg.length())
		throw ISerializeProto::DeserializeError(throw_msg);
	return this;
}
