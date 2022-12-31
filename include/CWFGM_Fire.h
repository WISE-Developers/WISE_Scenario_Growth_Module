/**
 * WISE_Scenario_Growth_Module: CWFGM_Fire.h
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

#include "FireEngine.h"
#include "ICWFGM_GridEngine.h"
#include "ISerializeProto.h"
#include "FireEngine_ext.h"
#include "semaphore.h"
#include "poly.h"
#include "WTime.h"
#include <set>
#include "cwfgmFire.pb.h"

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif

#ifndef DOXYGEN_IGNORE_CODE
class Ignition : public MinNode
{
public:
	Ignition();
	Ignition(const Ignition &toCopy);
	~Ignition();

	__INLINE Ignition *LN_Succ() const { return (Ignition *)MinNode::LN_Succ(); };
	__INLINE Ignition *LN_Pred() const { return (Ignition *)MinNode::LN_Pred(); };

	XY_Poly						*m_ignition;
	std::uint16_t				m_ignitionPolyType;
	std::vector<GDAL_Attribute>	m_attributes;

	DECLARE_OBJECT_CACHE_MT(Ignition, Ignition)
};
#endif


class SerializeIgnitionData : public ISerializationData
{
public:
	std::vector<std::string> *permissible_drivers;
};


/**
	This COM object stores values for each ignition. Attributes of an ignition that are not of concern to the simulation
	engine, such as its name or comments, are the responsibility of the user interface and do not appear on the COM interface.
	Similarly, the application's management of ignitions is of no concern of the simulation engine, and the fire collection
	does not define a COM interface.  The client application (such as CWFGM) is expected to use object-oriented technologies
	such as class inheritance to add attributes that the simulation engine is not concerned with.\n\n

	This object also implements the standard COM IPersistStream, IPersistStreamInit, and IPersistStorage  interfaces, for use
	for loading and saving.  These operations save only the start time and ignition location for a fire, no simulation data can be recorded.
*/
class FIRECOM_API CCWFGM_Ignition : public ICWFGM_CommonBase, public ISerializeProto
{
public:
	CCWFGM_Ignition();
	CCWFGM_Ignition(const CCWFGM_Ignition &toCopy);
	~CCWFGM_Ignition();

public:
	/**
	 * This property identifies the start time (in GMT, count of seconds since Midnight January 1, 1600) for the fire.  Local time is calculated from data provided by the grid engine attached to a given scenario.  This value cannot be modified when the fire object is attached to a running simulation.
		\param pVal Value of IgnitionTime
		\sa ICWFGM_Ignition::GetIgnitionTime

		\retval E_POINTER The address provided for pVal is invalid
		\retval S_OK Successful
		\retval ERROR_SCENARIO_SIMULATION_RUNNING If trying to set a new start time when the fire is attached to a running simulation
	*/
	virtual NO_THROW HRESULT GetIgnitionTime(HSS_Time::WTime *pVal);
	/**
	 * This property identifies the start time (in GMT, count of seconds since Midnight January 1, 1600) for the fire.  Local time is calculated from data provided by the grid engine attached to a given scenario.  This value cannot be modified when the fire object is attached to a running simulation.
		\param newVal Replacement value for IgnitionTime
		\sa ICWFGM_Ignition::SetIgnitionTime

		\retval S_OK Successful
		\retval ERROR_SCENARIO_SIMULATION_RUNNING If trying to set a new start time when the fire is attached to a running simulation
	*/
	virtual NO_THROW HRESULT SetIgnitionTime(const HSS_Time::WTime &newVal);
	/**
		Creates a new ignition object with all the same properties of the object being called, returns a handle to the new object in fire.  This method does not attach the new fire to any scenarios that this object is attached to, nor does it simulate any fire growth to duplicate the calculated data that this object may have.
		\param fire An ignition object
		\sa ICWFGM_Ignition::Clone
		
		\retval E_POINTER The address provided for fire is invalid
		\retval S_OK Successful
	*/
	virtual NO_THROW HRESULT Clone(boost::intrusive_ptr<ICWFGM_CommonBase> *newObject) const override;
	/**
	 * Changes the state of the object with respect to access rights.  When the object is used by an active simulation, it must not be modified.
	 * When the object is somehow modified, it must be done so in an atomic action to prevent concerns with arising from multithreading.
	 * Note that these locks are primarily needed to ensure a simulation dependency is not changed while it occurs.  All routines are otherwise multi-thread safe.
		\param exclusive true if the requester wants a write lock, false for read/shared access
		\param obtain true to obtain the lock, false to release the lock.  If this is false, then the exclusive parameter must match the initial call to obtain the lock.
		\sa ICWFGM_Ignition::MT_Lock

		\retval S_OK Successful
		\retval SUCCESS_STATE_OBJECT_LOCKED_WRITE Exclusive/write lock obtained
		\retval SUCCESS_STATE_OBJECT_LOCKED_SCENARIO A scenario has a lock for purposes of simulating (shared/read)
		\retval SUCCESS_STATE_OBJECT_LOCKED_READ Shared/read lock obtained
	*/
	virtual NO_THROW HRESULT MT_Lock(bool exclusive, std::uint16_t obtain);
	virtual NO_THROW HRESULT Valid(const HSS_Time::WTime &start_time, const HSS_Time::WTimeSpan &duration);

	/**
	 * Retrieves the object exposing the GridEngine interface that this Ignition object may refer to, to use for tasks such as bounds clipping, etc.
		\param	pVal	Value of GridEngine.
		\sa ICWFGM_VectorEngine::GridEngine
		\retval	E_POINTER	The address provided for pVal is invalid, or upon setting pVal the pointer doesn't appear to belong to an object exposing the ICWFGM_GridEngine interface.
		\retval	S_OK	Successful.
		\retval	ERROR_GRID_UNINITIALIZED	The Grid Engine property has not be set.
	*/
	virtual NO_THROW HRESULT get_GridEngine(boost::intrusive_ptr<ICWFGM_GridEngine> *pVal);
	/**
	 * Sets the object exposing the GridEngine interface that this Ignition object may refer to, to use for tasks such as bounds clipping, etc.
		\param	newVal	Replacement value for GridEngine.
		\sa ICWFGM_VectorEngine::GridEngine
		\retval	E_POINTER	The address provided for pVal is invalid, or upon setting pVal the pointer doesn't appear to belong to an object exposing the ICWFGM_GridEngine interface.
		\retval	S_OK	Successful.
		\retval	ERROR_GRID_UNINITIALIZED	The Grid Engine property has not be set.
		\retval	E_NOINTERFACE	The object provided does not implement the ICWFGM_GridEngine interface.
	*/
	virtual NO_THROW HRESULT put_GridEngine(ICWFGM_GridEngine * newVal);
	virtual NO_THROW HRESULT put_CommonData(ICWFGM_CommonData* pVal);

	/** Adds an ignition to this ignition object.
		\param ignition_type Whether it's a point, line, or polygon ignition.  Valid values are:
				<ul>
				<li><code>CWFGM_FIRE_IGNITION_POINT</code>
				<li><code>CWFGM_FIRE_IGNITION_LINE</code>
				<li><code>CWFGM_FIRE_IGNITION_POLYGON_OUT</code>
				<li><code>CWFGM_FIRE_IGNITION_POLYGON_IN</code>
				</ul>
		\param xy_pairs 2D Array of coordinates
		\param index Index of the newly added sub-ignition
		\sa ICWFGM_Ignition::AddIgnition

		\retval S_OK Successful
		\retval ERROR_FIRE_IGNITION_TYPE_UNKNOWN An invalid ignition type has been specified
		\retval ERROR_SCENARIO_SIMULATION_RUNNING If this function is run while the scenario is running
		\retval E_INVALIDARG If the ignition size has been set to less than or equal to zero
		\retval ERROR_SCENARIO_FIRE_ALRADY_ADDED The fire being added is already associated with this scenario
		\retval E_OUTOFMEMORY Insufficient memory 
		\retval E_POINTER Address for xy_pairs is invalid
	*/
	virtual NO_THROW HRESULT AddIgnition(std::uint16_t ignition_type, const XY_PolyConst &xy_pairs, std::uint32_t *index);
	/** This method sets the ignition for this fire.  This data cannot be modified when the fire object is attached to a running simulation.  ignition_size states the number of ignition points to follow.  The array is specified as consecutive X, Y pairs.  The array sizes are defined by 2*ignition_size.  The units for x, y are in grid units, not in metres, etc.
		\param index Where to insert this sub-ignition
		\param ignition_type Whether it's a point, line, or polygon ignition.  Valid values are:
				<ul>
				<li><code>CWFGM_FIRE_IGNITION_POINT</code>
				<li><code>CWFGM_FIRE_IGNITION_LINE</code>
				<li><code>CWFGM_FIRE_IGNITION_POLYGON_OUT</code>
				<li><code>CWFGM_FIRE_IGNITION_POLYGON_IN</code>
				</ul>
		\param xy_pairs 2D Array of coordinates
		\sa ICWFGM_Ignition::SetIgnition

		\retval E_POINTER The address provided for xy_pairs is invalid
		\retval S_OK Successful
		\retval E_OUTOFMEMORY 
		\retval ERROR_SCENARIO_SIMULATION_RUNNING If the fire is attached to a running simulation
		\retval E_INVALIDARG If any of the x or y values appear to be invalid
	*/
	virtual NO_THROW HRESULT SetIgnition(std::uint32_t index, std::uint16_t ignition_type, const XY_PolyConst &xy_pairs);
	/** This method sets the ignition for this fire. This data cannot be modified when the fire object is attached to a running simulation. The arrray is in Well Known Binary. The units for x, y are in grid units, not in metres, etc. 
		\param index Where to insert this sub-ignition
		\param wkb An array of WKB bytes
		\sa CCWFGM_Ignition::SetIgnition
		*/
	virtual NO_THROW HRESULT SetIgnitionWKB(std::uint32_t index, const unsigned char *wkb);
	/** This method clears the sub-ignition from this ignition set. If index = (unsigned long) -1, then all ignitions are cleared.
		\param index Index of the sub-ignition to remove
		\sa ICWFGM_Ignition::ClearIgnition

		\retval S_OK Successful
		\retval ERROR_SCENARIO_SIMULATION_RUNNING If the fire is attached to a running simulation
		\retval E_INVALIDARG If there are no ignitions to clear
	*/
	virtual NO_THROW HRESULT ClearIgnition(std::uint32_t index);
	/** Returns the number of sub-ignitions associated with this ignition object.
		\param count Number of sub-ignitions
		\sa ICWFGM_Ignition::GetIgnitionCount

		\retval E_POINTER The address provided for size is invalid
		\retval S_OK Successful
	*/
	virtual NO_THROW HRESULT GetIgnitionCount(std::uint32_t *count);
	/** Returns the type of ignition whether it be point, line, or polygon.
		\param index Sub-ginition being queried
		\param ignition_type Return value.  Valid values are:
				<ul>
				<li><code>CWFGM_FIRE_IGNITION_POINT</code>
				<li><code>CWFGM_FIRE_IGNITION_LINE</code>
				<li><code>CWFGM_FIRE_IGNITION_POLYGON_OUT</code>
				<li><code>CWFGM_FIRE_IGNITION_POLYGON_IN</code>
				</ul>
		\sa ICWFGM_Ignition::GetIgnitionType

		\retval E_POINTER The address provided for ignition_type is invalid
		\retval E_INVALIDARG If there are no ignitions to get type from
		\retval S_OK Successful
	*/
	virtual NO_THROW HRESULT GetIgnitionType(std::uint32_t index, std::uint16_t *ignition_type);
	/** Returns the number of unique attribute names associated with this ignition object.
		\param count Number of sub-ignitions
		\sa ICWFGM_Ignition::GetIgnitionCount

		\retval E_POINTER The address provided for size is invalid
		\retval S_OK Successful
	*/
	virtual NO_THROW HRESULT GetIgnitionAttributeCount(std::uint32_t* count);
	virtual NO_THROW HRESULT GetIgnitionAttributeName(std::uint32_t count, std::string* attribute_name);
	virtual NO_THROW HRESULT GetIgnitionAttributeValue(std::uint32_t index, const std::string &attribute_name, GDALVariant* ignition_type);
	/** This method returns the number of vertices of the specified sub-ignition.  Or, if -1 is provided for index, then the maxumum number of vertices found for any of
		the sub-ignitions defined is returned.
		\param index Sub-ignition being queried, or -1 to query all sub-ignitions.
		\param ignition_size Return value
		\sa ICWFGM_Ignition::GetIgnitionSize

		\retval E_POINTER  The address provided for size is invalid
		\retval E_INVALIDARG If there are no ignitions to get type from
		\retval S_OK Successful
	*/
	virtual NO_THROW HRESULT GetIgnitionSize(std::uint32_t index, std::uint32_t *ignition_size);
	/** This method returns the size of the WKB array that will be generated from the sub-ignition at the specified index.
		\param index Index of sub-ignition
		\param size Size of array in bytes
		\sa CCWFGM_Ignition::GetIgnitionWKBSize
		*/
	virtual NO_THROW HRESULT GetIgnitionWKBSize(std::uint32_t index, std::uint64_t *ignition_size);
	/** This method returns the indices of the specified polygon sub-ignition at the specified index.  size is adjusted as necessary to return the number of array entries used.
		\param index Index of sub-ignition
		\param ignition_type Point, line, or poly ignition.  Valid values are:
				<ul>
				<li><code>CWFGM_FIRE_IGNITION_POINT</code>
				<li><code>CWFGM_FIRE_IGNITION_LINE</code>
				<li><code>CWFGM_FIRE_IGNITION_POLYGON_OUT</code>
				<li><code>CWFGM_FIRE_IGNITION_POLYGON_IN</code>
				</ul>
		\param xy_pairs 2D Array of coordinates
		\sa ICWFGM_Ignition::GetIgnition

		\retval E_POINTER The address provided for size, ignition_type, or xy_pairs is invalid
		\retval E_INVALIDARG If there are no ignitions to get type from
		\retval S_OK Successful
		\retval E_OUTOFMEMORY Insufficient memory
	*/
	virtual NO_THROW HRESULT GetIgnition(std::uint32_t index, std::uint16_t *ignition_type, XY_Poly *xy_pairs);
	virtual NO_THROW HRESULT GetIgnitionLength(std::uint32_t index, double* length);
	virtual NO_THROW HRESULT GetIgnitionCentroid(std::uint32_t index, XY_Point* xy);
	/** This method returns the Well Known Binary array of the sub-ignition at the specified index.
		\param index Index of sub-ignition
		\param wkb Array used to hold the WKB array
		\sa CCWFGM_Ignition::GetIgnitionWKB
		*/
	virtual NO_THROW HRESULT GetIgnitionWKB(std::uint32_t index, unsigned char **wkb);
	/**	This method returns the bounding box of a specific ignition, or of all ignitions in the object.
		\param	index	Index of polygon.  If the bounding box for all ignitions is desired, then pass (unsigned long)-1.
		\param	min_pt	Minimum coordinate.
		\param	max_pt	Maximum coordinate.
		\sa ICWFGM_Ignition::GetIgnitionRange
		\retval	S_OK	Successful.
		\retval	E_POINTER	Address provided for x_min, x_max, y_min, or y_max is invalid.
		\retval	E_INVALIDARG	Ignition does not exist.
	*/
	virtual NO_THROW HRESULT GetIgnitionRange(std::uint32_t index, XY_Point* min_pt, XY_Point* max_pt);
	/** Imports an ignition from the file path specified.
		\param file_path File to import
		\parampermissible_drivers	Array of drivers (known by GDAL) identifying the file types that are allowed to be imported.
		\param attribute_names	Any attributes in the imported SHP file that should be logged with the ignition
		\sa ICWFGM_Ignition::ImportIgnition

		\retval E_POINTER The address provided for file_path is invalid
		\retval E_INVALIDARG Cannot understand file_path
		\retval S_FALSE Unspecified failure
		\retval S_OK Successul
		\retval ERROR_SEVERITY_WARNING Unspecified failure
		\retval ERROR_IGNITION_UNINITIALIZED Specified grid is invalid or not provided
		\retval ERROR_FIRE_IGNITION_TYPE_UNKNOWN Unknown ignition type (not point, line, polygon)
	*/
	virtual NO_THROW HRESULT ImportIgnition(const std::string &file_name, const std::vector<std::string> *permissible_drivers, const std::vector<std::string>* attribute_name);
	/** Imports an ignition from the WFS URL specified.
		\param	url		Identifies the WFS provider.
		\param	layer	Identifies the layer in the WFS provider.
		\param	username Username for access, if needed.
		\param	password Password for the user, if needed.
		\sa ICWFGM_Ignition::ImportIgnitionWFS

		\retval E_POINTER The address provided for any of the parameters is invalid
		\retval E_INVALIDARG Cannot understand file_path
		\retval S_FALSE Unspecified failure
		\retval S_OK Successul
		\retval ERROR_SEVERITY_WARNING Unspecified failure
		\retval ERROR_GRID_UNINITIALIZED Specified grid is invalid or not provided
	*/
	virtual NO_THROW HRESULT ImportIgnitionWFS(const std::string &url, const std::string &layer, const std::string &username, const std::string &password);
	/** Exports the ignition to the specified file path.
		\param driver_name Determines the file format (refer to GDAL documentation for supported formats)
		\param bprojection Projection file
		\param file_path Path/file of the vector data
		\sa ICWFGM_Ignition::ExportIgnition

		\retval E_POINTER The address provided for driver_name, file_path, or projection is invalid
		\retval E_INVALIDARG Unspecified error
		\retval S_FALSE Unspecified error
		\retval S_OK Successful 
		\retval ERROR_SEVERITY_WARNING Unspecified failure
		\retval ERROR_IGNITION_UNINITIALIZED Specified grid is invalid or not provided
	*/
	virtual NO_THROW HRESULT ExportIgnition(const std::string &driver_name, const std::string &projection, const std::string &file_name);
	/** Exports the ignition to the specified file path.
		\param	url		Identifies the WFS provider.
		\param	layer	Identifies the layer in the WFS provider.
		\param	username Username for access, if needed.
		\param	password Password for the user, if needed.
		\sa ICWFGM_Ignition::ExportIgnitionWFS

		\retval E_POINTER The address provided for any of the parameters is invalid
		\retval E_INVALIDARG Unspecified error
		\retval S_FALSE Unspecified error
		\retval S_OK Successful 
		\retval ERROR_SEVERITY_WARNING Unspecified failure
		\retval ERROR_GRID_UNINITIALIZED Specified grid is invalid or not provided
	*/
	virtual NO_THROW HRESULT ExportIgnitionWFS(const std::string &url, const std::string &layer, const std::string &username, const std::string &password);
	/** Polymorphic.  This routine sets an attribute/option value given the attribute/option index.  Currently does not
		perform any operation and is simply reserved for future functionality.
		\param option Reserved for future functionality.
		\param value Value for the attribute/option index
		\sa ICWFGM_Ignition::SetAttribute

		\retval E_NOTIMPL This function is reserved for future functionality.
	*/
	virtual NO_THROW HRESULT SetAttribute(std::uint16_t option, const PolymorphicAttribute &value);
	/** Polymorphic.  This routine retrieves an attribute/option value given the attribute/option index.
		\param option Supported / valid attribute/option index supported are:
		<ul>
		<li><code>CWFGM_ATTRIBUTE_LOAD_WARNING</code>	BSTR.  Any warnings generated by the COM object when deserializating.
		</ul>
		\param value Return value for the attribute/option index
		\sa ICWFGM_Ignition::GetAttribute

		\retval E_POINTER The address provided for value is invalid
		\retval E_INVALIDARG No valid parameters
		\retval S_OK Successful
	*/
	virtual NO_THROW HRESULT GetAttribute(std::uint16_t option, PolymorphicAttribute *value);
#ifndef DOXYGEN_IGNORE_CODE
public:
	virtual std::int32_t serialVersionUid(const SerializeProtoOptions& options) const noexcept override;
	virtual WISE::FireEngineProto::CwfgmIgnition* serialize(const SerializeProtoOptions& options) override;
	virtual CCWFGM_Ignition *deserialize(const google::protobuf::Message& proto, std::shared_ptr<validation::validation_object> valid, const std::string& name) override { return deserialize(proto, valid, name, nullptr); }
	virtual CCWFGM_Ignition *deserialize(const google::protobuf::Message& proto, std::shared_ptr<validation::validation_object> valid, const std::string& name, ISerializationData* data) override;
	virtual std::optional<bool> isdirty(void) const noexcept override { return m_bRequiresSave; }

public:
	CRWThreadSemaphore		m_lock;
	boost::intrusive_ptr<ICWFGM_GridEngine>	m_gridEngine;
	HSS_Time::WTime			m_startTime;
	HSS_Time::WTimeManager	*m_timeManager;

	MinListTempl<Ignition>	m_ignitionList;
	std::string				m_loadWarning;
	double					m_resolution;
	std::set<std::string>	m_attributeNames;
	std::string				m_gisURL, m_gisLayer, m_gisUID, m_gisPWD;
	bool					m_bRequiresSave;

	HRESULT fixResolution();
#endif
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif
