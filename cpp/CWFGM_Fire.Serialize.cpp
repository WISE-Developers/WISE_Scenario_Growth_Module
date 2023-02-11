/**
 * WISE_Scenario_Growth_Module: CWFGM_Fire.Serialize.cpp
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
#include "propsysreplacement.h"
#include "FireEngine.h"
#include "CWFGM_Fire.h"
#include "GridCom_ext.h"
#include "results.h"
#include "WTime.h"
#include "poly.h"
#include "CoordinateConverter.h"
#include "gdalclient.h"
#include "url.h"
#include "geo_poly.h"
#include "filesystem.hpp"
#include <boost/scoped_ptr.hpp>
#include "doubleBuilder.h"
#include "str_printf.h"
#include <google/protobuf/any.h>


using namespace HSS_Time;


HRESULT CCWFGM_Ignition::ImportIgnition(const std::filesystem::path &file_path, const std::vector<std::string_view> &permissible_drivers, const std::vector<std::string>* attribute_names) {
	if (file_path.empty())							return ERROR_IGNITION_UNINITIALIZED;
	if (!m_gridEngine)								return ERROR_GRID_UNINITIALIZED;

	HRESULT hr;
	PolymorphicAttribute var;

	if (FAILED(hr = m_gridEngine->GetAttribute(0, CWFGM_GRID_ATTRIBUTE_SPATIALREFERENCE, &var)))	{ return hr; }
	std::string projection;
	try { projection = std::get<std::string>(var); } catch (std::bad_variant_access &) {
		weak_assert(false);
		return ERROR_PROJECTION_UNKNOWN;
	} /*POLYMORPHIC*/

	CSemaphoreEngage lock(GDALClient::GDALClient::getGDALMutex(), true);

	OGRSpatialReferenceH oSourceSRS = CCoordinateConverter::CreateSpatialReferenceFromWkt(projection.c_str());

	XY_PolyLLSetAttributes set;
	set.SetCacheScale(m_resolution);
	hr = set.ImportPoly(permissible_drivers, file_path, oSourceSRS, nullptr, nullptr, attribute_names);
	if (oSourceSRS)
		CCoordinateConverter::DestroySpatialReference(oSourceSRS);

	if (SUCCEEDED(hr)) {
		if (set.IsEmpty()) {
			return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
		}

		Ignition *ig, *iig;
		while (ig = m_ignitionList.RemHead())
			delete ig;
		m_attributeNames.clear();

		if (set.NumPolys() > 1) {
			XY_PolyLLAttributes *p = set.LH_Head();
			std::uint16_t flag = p->m_publicFlags & XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_POLYMASK;
			p = p->LN_Succ();
			while (p->LN_Succ()) {
				if ((p->m_publicFlags & XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_POLYMASK) != flag)
					return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
				p = p->LN_Succ();
			}
		}

		XY_PolyLLAttributes *p = set.LH_Head();
		while (p->LN_Succ()) {
			ig = new Ignition();
			if (p->IsMultiPoint()) {
				ig->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POINT;
				p->CleanPoly(0.0, XY_PolyLLAttributes::Flags::INTERPRET_MULTIPOINT);
			}
			else if (p->IsPolyline()) {
				ig->m_ignitionPolyType = CWFGM_FIRE_IGNITION_LINE;
				p->CleanPoly(0.0, XY_PolyLLAttributes::Flags::INTERPRET_POLYLINE);
			}
			else if (p->IsPolygon()) {
				p->CleanPoly(0.0, XY_PolyLLAttributes::Flags::INTERPRET_POLYGON);

				if (p->IsInterior()) {
					weak_assert(p->DetermineRotation() > 0);
					ig->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POLYGON_IN;
				} else {
					weak_assert(p->DetermineRotation() < 0);
					ig->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POLYGON_OUT;
					}
			}
			try {
				ig->m_ignition = new XY_Poly(*p);
			}
			catch (std::exception& e) {
				delete ig;
				return E_OUTOFMEMORY;
			}
			ig->m_attributes = p->m_attributes;
			for (auto a : ig->m_attributes) {
				m_attributeNames.insert(a.attributeName);
			}

			bool same = false;
			iig = m_ignitionList.LH_Head();
			while (iig->LN_Succ()) {
				if ((ig->m_ignitionPolyType == iig->m_ignitionPolyType) &&
				    (ig->m_ignition->Equals(*iig->m_ignition, 0))) {
					same = true;
					break;
				}
				iig = (Ignition *)iig->LN_Succ();
			}
			if (same)	delete ig;
			else		m_ignitionList.AddTail(ig);
			p = p->LN_Succ();
		}
		m_bRequiresSave = true;
	}
	return hr;
}


#ifndef DOXYGEN_IGNORE_CODE

static std::string prepareUri(const std::string& uri)
{
	remote::url u;
	u.setUrl(uri);
	u.addParam("SERVICE", "WFS");
	u.addParam("REQUEST", "GetCapabilities");
	return u.build();
}

#endif


HRESULT CCWFGM_Ignition::ImportIgnitionWFS(const std::string &url, const std::string &layer, const std::string &username, const std::string &password) {
	if (!url.length())											return E_INVALIDARG;
	if (!layer.length())										return E_INVALIDARG;
	if (!m_gridEngine)								return ERROR_GRID_UNINITIALIZED;

	HRESULT hr;
	PolymorphicAttribute var;

	if (FAILED(hr = m_gridEngine->GetAttribute(0, CWFGM_GRID_ATTRIBUTE_SPATIALREFERENCE, &var)))	{ return hr; }
	std::string projection;
	try { projection = std::get<std::string>(var); } catch (std::bad_variant_access &) { weak_assert(false); return ERROR_PROJECTION_UNKNOWN; }; /*POLYMORPHIC*/

	OGRSpatialReferenceH oSourceSRS = CCoordinateConverter::CreateSpatialReferenceFromWkt(projection.c_str());

	XY_PolyLLSet set;
	set.SetCacheScale(m_resolution);
	std::vector<std::string> layers;
	layers.push_back(layer);
	XY_PolyLLSet pset;
	std::string URI = prepareUri(url);
	const std::vector<std::string_view> drivers;
	hr = set.ImportPoly(drivers, URI.c_str(), oSourceSRS, nullptr, &layers);

	if (oSourceSRS)
		OSRDestroySpatialReference(oSourceSRS);

	if (SUCCEEDED(hr)) {
		if (set.IsEmpty()) {
			return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
		}

		Ignition *ig, *iig;
		while (ig = m_ignitionList.RemHead())
			delete ig;

		if (set.NumPolys() > 1) {
			XY_PolyLL *p = set.LH_Head();
			std::uint16_t flag = p->m_publicFlags & XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_POLYMASK;
			p = p->LN_Succ();
			while (p->LN_Succ()) {
				if ((p->m_publicFlags & XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_POLYMASK) != flag)
					return ERROR_FIRE_IGNITION_TYPE_UNKNOWN;
				p = p->LN_Succ();
			}
		}

		XY_PolyLL *p = set.LH_Head();
		while (p->LN_Succ()) {
			ig = new Ignition();
			if (p->IsMultiPoint())		ig->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POINT;
			else if (p->IsPolyline())	ig->m_ignitionPolyType = CWFGM_FIRE_IGNITION_LINE;
			else if (p->IsPolygon()) {
				if (p->DetermineRotation() > 0)
					ig->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POLYGON_IN;
				else	ig->m_ignitionPolyType = CWFGM_FIRE_IGNITION_POLYGON_OUT;
			}
			ig->m_ignition = new XY_Poly(*p);

			bool same = false;
			iig = m_ignitionList.LH_Head();
			while (iig->LN_Succ()) {
				if ((ig->m_ignitionPolyType == iig->m_ignitionPolyType) &&
				    (ig->m_ignition->Equals(*iig->m_ignition, 0))) {
					same = true;
					break;
				}
				iig = (Ignition *)iig->LN_Succ();
			}
			if (same)	delete ig;
			else		m_ignitionList.AddTail(ig);
			p = p->LN_Succ();
		}
		m_gisURL = url;
		m_gisLayer = layer;
		m_gisUID = username;
		m_gisPWD = password;

		m_bRequiresSave = true;
	}
	return hr;
}


HRESULT CCWFGM_Ignition::ExportIgnition(std::string_view driver_name, const std::string& bprojection, const std::filesystem::path &file_path) {
	if ((!driver_name.length()) || (file_path.empty()))
		return E_INVALIDARG;
	if (!m_gridEngine)								return ERROR_IGNITION_UNINITIALIZED;;

	CSemaphoreEngage lock(GDALClient::GDALClient::getGDALMutex(), true);

	HRESULT hr;
	OGRSpatialReferenceH oTargetSRS = CCoordinateConverter::CreateSpatialReferenceFromStr(bprojection.c_str());

	PolymorphicAttribute var;

	XY_PolyLLSet set;
	set.SetCacheScale(m_resolution);

	Ignition *ig = m_ignitionList.LH_Head();
	while (ig->LN_Succ()) {
		XY_PolyLL *p = new XY_PolyLL(*ig->m_ignition);
		if (ig->m_ignitionPolyType == CWFGM_FIRE_IGNITION_POINT)
			p->m_publicFlags |= XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_MULTIPOINT;
		else if (ig->m_ignitionPolyType == CWFGM_FIRE_IGNITION_LINE)
			p->m_publicFlags |= XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_POLYLINE;
		else
			p->m_publicFlags |= XY_PolyLL_BaseTempl<double>::Flags::INTERPRET_POLYGON;
		set.AddPoly(p);
		ig = (Ignition *)ig->LN_Succ();
	}

	if (FAILED(hr = m_gridEngine->GetAttribute(0, CWFGM_GRID_ATTRIBUTE_SPATIALREFERENCE, &var))) return hr;
	std::string projection;
	try { projection = std::get<std::string>(var); } catch (std::bad_variant_access &) { weak_assert(false); return ERROR_PROJECTION_UNKNOWN; }; /*POLYMORPHIC*/

	OGRSpatialReferenceH oSourceSRS = CCoordinateConverter::CreateSpatialReferenceFromWkt(projection.c_str());

	hr = set.ExportPoly(driver_name, file_path, oSourceSRS, oTargetSRS);
	if (oSourceSRS)
		CCoordinateConverter::DestroySpatialReference(oSourceSRS);
	if (oTargetSRS)
		CCoordinateConverter::DestroySpatialReference(oTargetSRS);
	return hr;
}


HRESULT CCWFGM_Ignition::ExportIgnitionWFS(const std::string &url, const std::string &layer, const std::string &username, const std::string &password) {
	return E_NOTIMPL;
}


std::int32_t CCWFGM_Ignition::serialVersionUid(const SerializeProtoOptions& options) const noexcept {
	return options.fileVersion();
}


WISE::FireEngineProto::CwfgmIgnition* CCWFGM_Ignition::serialize(const SerializeProtoOptions& options)
{
	auto engine = new WISE::FireEngineProto::CwfgmIgnition();
	engine->set_version(serialVersionUid(options));

	engine->set_allocated_starttime(HSS_Time::Serialization::TimeSerializer().serializeTime(m_startTime, options.fileVersion()));

	auto ignition = m_ignitionList.LH_Head();
	auto list = new WISE::FireEngineProto::CwfgmIgnition_IgnitionList();
	while (ignition->LN_Succ())
	{
		auto i = list->add_ignitions();
		i->set_polytype((WISE::FireEngineProto::CwfgmIgnition_IgnitionPoint_IgnitionShape)(ignition->m_ignitionPolyType / 100));

		GeoPoly geo(ignition->m_ignition);
		geo.setStoredUnits(GeoPoly::UTM);

		for (auto& a : ignition->m_attributes)
		{
			auto attr = i->add_attributes();
			attr->set_name(a.attributeName);

			google::protobuf::Message* msg = nullptr;
			if (std::holds_alternative<std::int32_t>(a.attributeValue))
				msg = createProtobufObject(std::get<std::int32_t>(a.attributeValue));
			else if (std::holds_alternative<std::int64_t>(a.attributeValue))
				msg = createProtobufObject(std::get<std::int64_t>(a.attributeValue));
			else if (std::holds_alternative<double>(a.attributeValue))
				msg = createProtobufObject(std::get<double>(a.attributeValue));
			else if (std::holds_alternative<std::string>(a.attributeValue))
				msg = createProtobufObject(std::get<std::string>(a.attributeValue));

			if (msg)
			{
				attr->mutable_value()->PackFrom(*msg);
				delete msg;
			}
		}

		i->set_allocated_polygon(geo.getProtobuf(options.useVerboseFloats()));

		ignition = ignition->LN_Succ();
	}
	engine->set_allocated_ignitions(list);

	return engine;
}

CCWFGM_Ignition *CCWFGM_Ignition::deserialize(const google::protobuf::Message& proto, std::shared_ptr<validation::validation_object> valid, const std::string& name, ISerializationData* data)
{
	auto sdata = dynamic_cast<SerializeIgnitionData*>(data);
	auto engine = dynamic_cast_assert<const WISE::FireEngineProto::CwfgmIgnition*>(&proto);

	if (!engine)
	{
		if (valid)
			/// <summary>
			/// The object passed as a ignition is invalid. An incorrect object type was passed to the parser.
			/// </summary>
			/// <type>internal</type>
			valid->add_child_validation("WISE.FireEngineProto.CwfgmIgnition", name, validation::error_level::SEVERE, validation::id::object_invalid, proto.GetDescriptor()->name());
		weak_assert(false);
		m_loadWarning = "Error: WISE.FireEngineProto.CwfgmIgnition: Protobuf object invalid";
		throw ISerializeProto::DeserializeError("WISE.FireEngineProto.CwfgmFire: Protobuf object invalid", ERROR_PROTOBUF_OBJECT_INVALID);
	}

	if ((engine->version() != 1) && (engine->version() != 2))
	{
		if (valid)
			/// <summary>
			/// The object version is not supported. The ignition is not supported by this version of Prometheus.
			/// </summary>
			/// <type>user</type>
			valid->add_child_validation("WISE.FireEngineProto.CwfgmIgnition", name, validation::error_level::SEVERE, validation::id::version_mismatch, std::to_string(engine->version()));
		weak_assert(false);
		m_loadWarning = "Error: WISE.FireEngineProto.CwfgmIgnition: Version is invalid";
		throw ISerializeProto::DeserializeError("WISE.FireEngineProto.CwfgmFire: Version is invalid", ERROR_PROTOBUF_OBJECT_VERSION_INVALID);
	}

	PolymorphicAttribute var;
	if (FAILED(m_gridEngine->GetAttribute(0, CWFGM_GRID_ATTRIBUTE_SPATIALREFERENCE, &var))) {
		if (valid)
			/// <summary>
			/// The projection is not readable but should be by this time in deserialization.
			/// </summary>
			/// <type>internal</type>
			valid->add_child_validation("WISE.FireEngineProto.CwfgmIgnition", name, validation::error_level::SEVERE, validation::id::initialization_incomplete, "projection");
		m_loadWarning = "Error: WISE.FireEngineProto.CwfgmIgnition: Incomplete initialization";
		throw ISerializeProto::DeserializeError("WISE.FireEngineProto.CwfgmFire: Incomplete initialization");
	}

	std::string projection;
	projection = std::get<std::string>(var);

	boost::scoped_ptr<CCoordinateConverter> convert(new CCoordinateConverter());
	convert->SetSourceProjection(projection.c_str());

	auto vt = validation::conditional_make_object(valid, "WISE.FireEngineProto.CwfgmIgnition", name);
	auto v = vt.lock();

	if (engine->has_starttime())
	{
		auto time = HSS_Time::Serialization::TimeSerializer().deserializeTime(engine->starttime(), m_timeManager, nullptr, "");
		if (time) {
			m_startTime = HSS_Time::WTime(*time, m_startTime.GetTimeManager());
			delete time;
		}
	}
	m_startTime.PurgeToSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	if (m_startTime.GetTotalMicroSeconds() == 0)
	{
		m_loadWarning = "Error: WISE.FireEngineProto.CwfgmIgnition: Invalid ignition start time";
		if (v)
			v->add_child_validation("HSS.Times.WTime", "startTime", validation::error_level::WARNING,
				"test", m_startTime.ToString(WTIME_FORMAT_STRING_ISO8601),
				{ true, WTime::GlobalMin().ToString(WTIME_FORMAT_STRING_ISO8601) },
				{ true, WTime::GlobalMax().ToString(WTIME_FORMAT_STRING_ISO8601) });
	}
	if (m_startTime.GetMicroSeconds(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST)) {
		m_loadWarning += "Warning: fractions of seconds on the start time will be purged to the start of the minute.";
		if (v)
			/// <summary>
			/// The start time contains fractions of seconds.
			/// </summary>
			/// <type>user</type>
			v->add_child_validation("HSS.Times.WTime", "startTime", validation::error_level::WARNING, validation::id::time_invalid, m_startTime.ToString(WTIME_FORMAT_STRING_ISO8601), "Fractions of seconds will be purged.");
		m_startTime.PurgeToSecond(WTIME_FORMAT_AS_LOCAL | WTIME_FORMAT_WITHDST);
	}

	if (engine->data_case() == WISE::FireEngineProto::CwfgmIgnition::kIgnitions)
	{
		/// <summary>
		/// Child validations for set of ignitions.
		/// </summary>
		auto vt2 = validation::conditional_make_object(v, "IgnitionList", "ignitions");
		auto v2 = vt2.lock();

		for (int i = 0; i < engine->ignitions().ignitions_size(); i++)
		{
			/// <summary>
			/// Child validations for an individual ignition.
			/// </summary>
			auto vt3 = validation::conditional_make_object(v2, "IgnitionPoint", strprintf("ignitions[%d]", i));
			auto v3 = vt3.lock();

			auto ignition = engine->ignitions().ignitions(i);
			auto ig = new Ignition();
			ig->m_ignitionPolyType = ignition.polytype() * 100;
			if (ignition.has_polygon())
			{
				GeoPoly geo(ignition.polygon());
				geo.setStoredUnits(GeoPoly::UTM);
				geo.setConverter([&convert](std::uint8_t type, double x, double y, double z) -> std::tuple<double, double, double>
				{
					XY_Point loc = convert->start()
						.fromPoints(x, y, z)
						.asLatLon()
						.endInUTM()
						.to2DPoint();
					return std::make_tuple(loc.x, loc.y, 0.0);
				});
				ig->m_ignition = geo.getPolygon(true, v3, "polygon");
			}

			for (auto& a : ignition.attributes())
			{
				if (a.has_value())
				{
					if (a.value().Is<google::protobuf::Int32Value>())
					{
						m_attributeNames.insert(a.name());
						GDAL_Attribute attr;
						attr.attributeName = a.name();
						google::protobuf::Int32Value value;
						a.value().UnpackTo(&value);
						attr.attributeValue = value.value();
						ig->m_attributes.push_back(attr);
					}
					else if (a.value().Is<google::protobuf::Int64Value>())
					{
						m_attributeNames.insert(a.name());
						GDAL_Attribute attr;
						attr.attributeName = a.name();
						google::protobuf::Int64Value value;
						a.value().UnpackTo(&value);
						attr.attributeValue = value.value();
						ig->m_attributes.push_back(attr);
					}
					else if (a.value().Is<google::protobuf::DoubleValue>())
					{
						m_attributeNames.insert(a.name());
						GDAL_Attribute attr;
						attr.attributeName = a.name();
						google::protobuf::DoubleValue value;
						a.value().UnpackTo(&value);
						attr.attributeValue = value.value();
						ig->m_attributes.push_back(attr);
					}
					else if (a.value().Is<google::protobuf::StringValue>())
					{
						m_attributeNames.insert(a.name());
						GDAL_Attribute attr;
						attr.attributeName = a.name();
						google::protobuf::StringValue value;
						a.value().UnpackTo(&value);
						attr.attributeValue = value.value();
						ig->m_attributes.push_back(attr);
					}
					else {
						if (v3)
							v3->add_child_validation("IgnitionAttribute", strprintf("attributes[%d]", i), validation::error_level::SEVERE, validation::id::object_invalid, a.SerializeAsString());
					}
				}
				else {
					if (v3)
						v3->add_child_validation("IgnitionAttribute", strprintf("attributes[%d]", i), validation::error_level::SEVERE, validation::id::object_invalid, "[no value]");
				}
			}

			m_ignitionList.AddTail(ig);
		}
	}
	else if (engine->data_case() == WISE::FireEngineProto::CwfgmIgnition::kFilename)
	{
		if (!engine->filename().length()) {
			if (v)
				v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::missing_filename, engine->filename());
		}
		//import the file if it exists
		else if (fs::exists(fs::relative(engine->filename())) && sdata)
		{
			std::vector<std::string> attributes;
			bool valid = false;
			for (int i = 0; i < engine->file_attributes_size(); i++) {
				valid = true;
				std::string a = engine->file_attributes(i);
				attributes.push_back(a);
			}

			HRESULT hr;
			std::filesystem::path fname(engine->filename());
			hr = ImportIgnition(fname, *sdata->permissible_drivers, (valid) ? &attributes : nullptr);

			if (FAILED(hr)) {
				switch (hr) {
				case E_POINTER:							if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::e_pointer, engine->filename()); break;
				case E_INVALIDARG:						if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::e_invalidarg, engine->filename()); break;
				case ERROR_IGNITION_UNINITIALIZED:		if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::object_invalid, engine->filename()); break;
				case ERROR_FIRE_IGNITION_TYPE_UNKNOWN:	if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::enum_invalid, engine->filename()); break;
				case E_OUTOFMEMORY:						if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::out_of_memory, engine->filename()); break;
				case ERROR_FILE_NOT_FOUND:				if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::file_not_found, engine->filename()); break;
				case ERROR_TOO_MANY_OPEN_FILES:			if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::too_many_open_files, engine->filename()); break;
				case ERROR_ACCESS_DENIED:				if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::access_denied, engine->filename()); break;
				case ERROR_INVALID_HANDLE:				if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::invalid_handle, engine->filename()); break;
				case ERROR_HANDLE_DISK_FULL:			if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::disk_full, engine->filename()); break;
				case ERROR_FILE_EXISTS:					if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::file_exists, engine->filename()); break;
				case S_FALSE:
				case E_FAIL:
				case ERROR_SEVERITY_WARNING:
				default:
					if (v) v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::unspecified, engine->filename()); break;
				}
			}
		}
		else {
			if (v)
				v->add_child_validation("string", "filename", validation::error_level::SEVERE, validation::id::file_not_found, engine->filename());
		}
	}
	else {
		if (v)
			v->add_child_validation("string", "data", validation::error_level::SEVERE, validation::id::oneof_invalid, std::to_string(engine->data_case()));
	}

	return this;
}
