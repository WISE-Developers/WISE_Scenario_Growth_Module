/**
 * WISE_Scenario_Growth_Module: CWFGM_Scenario.h
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
#include "objectcache.h"
#include "WTime.h"
#include "semaphore.h"
#include "FwiCom.h"
#include "ICWFGM_Fuel.h"
#include "ICWFGM_GridEngine.h"
#include "ICWFGM_VectorEngine.h"
#include "ICWFGM_Asset.h"
#include "ICWFGM_Target.h"
#include "ScenarioAsset.h"
#include "StopCondition.h"
#include "ISerializeProto.h"
#include "CWFGM_Fire.h"
#include "FireEngine.h"
#include "ScenarioIgnition.h"
#include "Percentile.h"
#include "GustingOptions.h"
#include "cwfgmScenario.pb.h"
#include "hss_propagate_const.h"


using namespace HSS_Time;

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif

#ifndef DOXYGEN_IGNORE_CODE
class VectorEngineNode : public MinNode {
    public:
	VectorEngineNode* LN_Succ() const	{ return (VectorEngineNode*)MinNode::LN_Succ(); };
	VectorEngineNode* LN_Pred() const	{ return (VectorEngineNode*)MinNode::LN_Pred(); };

	boost::intrusive_ptr<ICWFGM_VectorEngine>		m_vectorEngine;
};
#endif


struct XYStat
{
	unsigned short stat;
	NumericVariant value;
};


/** Array when retrieving a set of statistics at a given location

	Structure defining various options for export SHP and KML files (like names, values, etc.).
*/
struct XYStatOptions {
	std::uint64_t mintime;				/* optional, used to define ranges */
	std::uint64_t time;					/* time at which to calculate statistics */
	unsigned long interp_method;			/* approach to interpolation */
	unsigned short discretization;			/* value for the statistic (returned) */
	unsigned short excludeIgnitionInteriors;	/* if set to true, then we don't try calculate any stat inside a polygon ignition */
};


class FIRECOM_API CriticalPathPointData : public XY_PolyNodeAttribute<double> {
public:
	CriticalPathPointData() = default;
	CriticalPathPointData(const CriticalPathPointData& c);

	CriticalPathPointData* LN_Succ() const { return (CriticalPathPointData*)XY_PolyNodeAttribute<double>::LN_Succ(); }
	CriticalPathPointData* LN_Pred() const { return (CriticalPathPointData*)XY_PolyNodeAttribute<double>::LN_Pred(); }

	DECLARE_OBJECT_CACHE_MT(CriticalPathPointData, CriticalPathPointData)
};


class FIRECOM_API CriticalPathPoint : public XY_PolyLLAttributes {
public:
	CriticalPathPoint();
	CriticalPathPoint(const CriticalPathPoint& toCopy);
	virtual ~CriticalPathPoint() override;

	virtual CriticalPathPointData* New() const override;
	virtual CriticalPathPointData* NewCopy(const XY_PolyLLNode<double>& toCopy) const override;
	virtual void Delete(XY_PolyLLNode<double>* c) const override;

	CriticalPathPoint* LN_Succ() const { return (CriticalPathPoint*)XY_PolyLLAttributes::LN_Succ(); }
	CriticalPathPoint* LN_Pred() const { return (CriticalPathPoint*)XY_PolyLLAttributes::LN_Pred(); }

	DECLARE_OBJECT_CACHE_MT(CriticalPathPoint, CriticalPathPoint)
};


class FIRECOM_API CriticalPath : public MinNode, public XY_PolyLLSetAttributes {
public:
	CriticalPath();
	CriticalPath(const CriticalPath& toCopy);
	virtual ~CriticalPath() override;

	CriticalPath* LN_Succ() const { return (CriticalPath*)MinNode::LN_Succ(); }
	CriticalPath* LN_Pred() const { return (CriticalPath*)MinNode::LN_Pred(); }

	virtual CriticalPathPoint* New() const override;
	virtual CriticalPathPoint* NewCopy(const XY_PolyLL_BaseTempl<double>& c) const override;
	virtual void Delete(XY_PolyLL_BaseTempl<double>* c) const override;
	void Remove(CriticalPathPoint* c) { MinList::Remove(c); }

	DECLARE_OBJECT_CACHE_MT(CriticalPath, CriticalPath)
};


template<class _type>
class Scenario;


/** Primary simulation scenario object

	The simulation engine implements and exposes a large scenario COM interface for CWFGM and other client applications to use.  The purposes of this object are as follows:
		- to provide mechanisms to model a fire (statistical information can be retrieved as well);
		- to provide the client appication (such as Prometheus) with a means to examine and set various parameters that affect the fire growth, such as acceleration on or off, etc.; and
		- to provide the client application with mechanisms to manipulate the objects that participate in the fire simulation, such as ignition points.

		This object also implements the standard COM IPersistStream, IPersistStreamInit, and IPersistStorage  interfaces, for use for loading and saving.  Serialization methods
	save scenario settings such as start time, duration, etc., but do not save any other associated objects, such as the grid engine, vector engines, or fire objects attached
	to it.  The client application is responsible for maintaining these settings.  This rule is imposed since the client object is responsible for maintaining collections of ignition
	objects, and objects exporting the ICWFGM_GridEngine interface.

	Presently, this interface defines synchronous access to simplify the simulation engine and the client application.  The client application makes a request Step() and will wait
	until the simulation engine is finished completing the request.  This model will provide sufficient control over the engine for this application.
*/
class FIRECOM_API CCWFGM_Scenario : public ICWFGM_CommonBase, /*public ISerializeXMLStream,*/ public ICWFGM_PercentileAttribute, public ISerializeProto {
public:
	friend class CWFGM_ScenarioHelper;
	template<typename T> friend class ScenarioTimeStep;
	template<typename T> friend class ScenarioCache;
	template<typename T> friend class Scenario;
	template<typename T> friend class FirePoint;
public:
	CCWFGM_Scenario();
	CCWFGM_Scenario(const CCWFGM_Scenario &toCopy);
	~CCWFGM_Scenario();

public:
	/** Creates a new scenario object with all the same properties of the object being
		called, returns a handle to the new object in 'scenario'.  This operation does not
		link the cloned object to the fires attached to the object being called, nor does it
		record the GridEngine handle in the new object, it only duplicates the properties
		such as start time, end time, calculation intervals, etc.
		\param scenario A scenario object
		\sa ICWFGM_Scenario::Clone

		\retval E_POINTER The address provided for scenario return value is invalid
		\retval S_OK Successful
		\retval E_OUTOFMEMORY Insufficient memory
		\retval ERROR_SEVERITY_WARNING Unspecified failure
		\retval E_NOINTERFACE Internal serialization issue/error
	*/
	virtual NO_THROW HRESULT Clone(boost::intrusive_ptr<ICWFGM_CommonBase> *newObject) const override;
	/** Sets or retrieves the object exposing the GridEngine interface that this scenario object uses to retrieve the spatial and temporal grid data (representing fuels, elevations, and weather).  Note that objects implementing this interface may be chained, or layered together, but the scenario object is only concerned with the top-most object.
		\param pVal Value of GridEngine
		\param	layerThread		Handle for scenario layering/stack access, allocated from an ICWFGM_LayerManager COM object.  Needed.  It is designed to allow nested layering analogous to the GIS layers.
		\sa ICWFGM_Scenario::GetGridEngine

		\retval E_POINTER The address provided for layerThread or pVal is invalid
		\retval S_OK Successful
		\retval ERROR_GRID_UNINITIALIZED The value was retrieved before it had been initialized.
	*/
	virtual NO_THROW HRESULT GetGridEngine(Layer **layerThread, boost::intrusive_ptr<ICWFGM_GridEngine> *pVal) const;
	/** Sets or retrieves the object exposing the GridEngine interface that this scenario object uses to retrieve the spatial and temporal grid data (representing fuels, elevations, and weather).  Note that objects implementing this interface may be chained, or layered together, but the scenario object is only concerned with the top-most object.
		\param newVal Replacement value for GridEngine
		\param	layerThread		Handle for scenario layering/stack access, allocated from an ICWFGM_LayerManager COM object.  Needed.  It is designed to allow nested layering analogous to the GIS layers.
		\sa ICWFGM_Scenario::PutGridEngine

		\retval S_OK Successful
		\retval ERROR_GRID_UNINITIALIZED The value was retrieved before it had been initialized.
		\retval E_NOINTERFACE The object provided doesn't support the ICWFGM_GridEngine interface.
	*/
	virtual NO_THROW HRESULT PutGridEngine(Layer *layerThread, ICWFGM_GridEngine *newVal);
	virtual NO_THROW HRESULT PutCommonData(Layer* layerThread, ICWFGM_CommonData* pVal);

	/** Returns the number of ignitions associated with this scenario object.
		\param count Number of ignitions
		\sa ICWFGM_Scenario::GetIgnitionCount

		\retval E_POINTER The address provided for count is invalid
		\retval S_OK Successful
	*/
	virtual NO_THROW HRESULT GetIgnitionCount(std::uint32_t *count) const;
	/** Adds an ignition to this scenario object.
		\param fire An ignition object
		\sa ICWFGM_Scenario::AddIgnition

		\retval S_OK Successful
		\retval E_OUTOFMEMORY Insufficient memory
		\retval ERROR_SCENARIO_BAD_STATE If this function is run while the scenario is running
		\retval E_NOINTERFACE The object trying to be added doesn't support the correct interfaces, or if the pointer to fire is invalid
		\retval ERROR_SCENARO_FIRE_ALREADY_ADDED The fire being added is already associated with this scenario
		\retval E-POINTER The address provided for fire is invalid.
	*/
	virtual NO_THROW HRESULT AddIgnition(CCWFGM_Ignition *fire);
	/** Removes an association between an ignition and this scenario object.
		\param fire An ignition object
		\sa ICWFGM_Scenario::RemoveIgnition

		\retval E_POINTER The address provided for fire is invalid
		\retval S_OK Successful
		\retval ERROR_SCENARIO_BAD_STATE If this function is run while the scenario is running
		\retval ERROR_SCENARIO_FIRE_UNKNOWN Fire is unknown to this scenario object
	*/
	virtual NO_THROW HRESULT RemoveIgnition(CCWFGM_Ignition *fire);
	/** Returns the index of fire in this scenario's set of ignitions.
		\param fire An ignition object
		\param index Index to a fire object
		\sa ICWFGM_Scenario::IndexOfIgnition

		\retval E_POINTER The address provided for fire or index is invalid
		\retval S_OK Successful
		\retval ERROR_SCENARIO_FIRE_UNKNOWN If fire does not resolve to a know fire for this scenario
	*/
	virtual NO_THROW HRESULT IndexOfIgnition(const CCWFGM_Ignition *fire, std::uint32_t *index) const;
	/** Given an index value, returns a pointer to an ignition associated with this scenario.
		\param index Index to an ignition
		\param fire An ignition object
		\sa ICWFGM_Scenario::IgnitionAtIndex

		\retval E_POINTER The address provided for fire is invalid
		\retval S_OK Successful
		\retval ERROR_SCENARIO_FIRE_UNKOWN Index is invalid
	*/
	virtual NO_THROW HRESULT IgnitionAtIndex(std::uint32_t index, boost::intrusive_ptr<CCWFGM_Ignition> *fire) const;

	/** Returns the number of vector engine objects associated with this scenario object.
		\param count Number of vector engine objects
		\sa ICWFGM_Scenario::GetVectorEngineCount

		\retval E_POINTER The address provided for count is invalid
		\retval S_OK Successful
	*/
	virtual NO_THROW HRESULT GetVectorEngineCount(std::uint32_t *count) const;
	/** Adds a vector engine object to this scenario object.  The order of vector engine objects is unimportant (where the order of grid engine objects is).
		\param vectorEngine A vector engine object.
		\sa ICWFGM_Scenario::AddVectorEngine

		\retval S_OK Successful
		\retval E_OUTOFMEMORY Insufficient memory
		\retval ERROR_SCENARIO_BAD_STATE If this function is run while the scenarion is running.
		\retval E_POINTER vectorEngine is invalid
		\retval E_NOINTERFACE VectorEngine does not support the ICWFGM_VectorEngine interface
		\retval ERROR_SCENARIO_VECTORENGINE_ALREADY_ADDED The vector engine object being added is already associated with this scenario.
	*/
	virtual NO_THROW HRESULT AddVectorEngine(ICWFGM_VectorEngine *vectorEngine);
	/** Removes a vector engine object from this scenario.
		\param vectorEngine A vector engine object
		\sa ICWFGM_Scenario::RemoveVectorEngine

		\retval E_POINTER The address provided for VectorEngine is invalid.
		\retval S_OK Successful
		\retval ERROR_SCENARIO_BAD_STATE If this function is run while the scenario is running
		\retval ERROR_SCENARIO_VECTORENGINE_UNKNOWN If vectorEngine does not resolve to a known vector engine for this scenario.
	*/
	virtual NO_THROW HRESULT RemoveVectorEngine(ICWFGM_VectorEngine *vectorEngine);
	/** Returns the index of vectorEngine in this scenario's set of vector engines.
		\param vectorEngine A vector engine object
		\param index Indes to a vector engine
		\sa ICWFGM_Scenario::IndexOfVectorEngine

		\retval E_POINTER The address provided for vectorEngine or index is invalid
		\retval S_OK Successful
		\retval ERROR_SCENARIO_VECTORENGINE_UNKNOWN If vectorEngine does not reolve to a known vector engine for this scenario
	*/
	virtual NO_THROW HRESULT IndexOfVectorEngine(const ICWFGM_VectorEngine *vectorEngine, std::uint32_t *index) const;
	/** Given an index value, returns a pointer to a vector engine associated with this scenario.
		\param index Indes to a vector engine
		\param vectorEngine A vector engine object
		\sa ICWFGM_Scenario::VectorEngineAtIndex

		\retval E_POINTER The address provided for vector Engine is invalid
		\retval S_OK Successful
		\retval ERROR_SCENARIO_VECTORENGINE_UNKNOWN If index does not resolve to a known vector engine
	*/
	virtual NO_THROW HRESULT VectorEngineAtIndex(const std::uint32_t index, boost::intrusive_ptr<ICWFGM_VectorEngine> *vectorEngine) const;

	virtual NO_THROW HRESULT GetAssetCount(std::uint32_t* count) const;
	virtual NO_THROW HRESULT AddAsset(ICWFGM_Asset* asset, std::uint32_t operation);
	virtual NO_THROW HRESULT RemoveAsset(ICWFGM_Asset* asset);
	virtual NO_THROW HRESULT IndexOfAsset(const ICWFGM_Asset* asset, std::uint32_t* index) const;
	virtual NO_THROW HRESULT AssetAtIndex(std::uint32_t index, boost::intrusive_ptr<ICWFGM_Asset>* asset) const;
	virtual NO_THROW HRESULT SetAssetOperation(ICWFGM_Asset* asset, std::uint32_t mode);
	virtual NO_THROW HRESULT GetAssetOperation(ICWFGM_Asset* asset, std::uint32_t* mode) const;
	virtual NO_THROW HRESULT GetAssetTimeCount(ICWFGM_Asset* asset, std::uint32_t* count) const;
	virtual NO_THROW HRESULT GetAssetTime(const ICWFGM_Asset* asset, const std::uint32_t index, bool* arrived, WTime* time) const;

	virtual NO_THROW HRESULT SetWindTarget(ICWFGM_Target* target, unsigned long index, unsigned long sub_index);
	virtual NO_THROW HRESULT GetWindTarget(ICWFGM_Target** target, unsigned long* index, unsigned long* sub_index) const;
	virtual NO_THROW HRESULT ClearWindTarget();
	virtual NO_THROW HRESULT SetVectorTarget(ICWFGM_Target* target, unsigned long index, unsigned long sub_index);
	virtual NO_THROW HRESULT GetVectorTarget(ICWFGM_Target** target, unsigned long* index, unsigned long* sub_index);
	virtual NO_THROW HRESULT ClearVectorTarget();

	/** Returns the current state of the simulation, being currently running, complete, or in some invalid state preventing any simulation from beginning.
		\sa ICWFGM_Scenario::Simulation_State
		\retval S_OK Scenario ready toexecute, but has not begun
		\retval ERROR_GRID_UNINITALIZED No ICWFGM_GridEngine object has been specified or that object doesn't have an initialized latitude, longitude, or time zone
		\retval ERROR_SCENARIO_NO_FIRES No fires have been attached to the scenario
		\retval ERROR_SCENARIO_BAD_TIMES Start of end time is invalid, or the internal or output calculations intervals are invalid
		\retval ERROR_SCEARNIO_BAD_TIMESTEPS The time steps are invalid
		\retval ERROR_SCENARIO_INVALID_BURNCONDITIONS	Invalid burn condition settings (burning periods between days overlap)
		\retval ERROR_SEVERITY_WARNING The FuelMap for the object implementing the ICWFGM_GridEngine interface has not been set, or no FBP data has been loaded
		\retval ERROR_GRID_WEATHER_NOT_IMPLEMENTED The object implementing the ICWFGM_GridEngine does not handle weather data, and it was asked to test for validity against simulation times
		\retval SUCCESS_SCENARIO_SIMULATION_RESET The scenarios has been reset and is pending simulation execution
		\retval SUCCESS_SCENARIO_SIMULATION_RUNNING The scenario's simulation is running
		\retval SUCCESS_SCENARIO_SIMULATION_COMPLETE The scenario's simulation is complete
		\retval SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS The scenario's simulation is complete because the edge of the grid boundary has been met
		\retval SUCCESS_SCENARIO_SIMULATION_COMPLETE_ASSET The scenario's simulation is complete because the correct number of assets have been encountered by the simulated fire(s)
		\retval SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION The scenario's simulation is complete because of 1 or more user defined conditions based on fire growth, saying it's complete
		\retval ERROR_GRID_WEATHER_INVALID_DATES Weather grids which were attached are in an invalid state
		\retval ERROR_GRID_PRIMARY_STREAM_UNSPECIFIED	Multiple weather streams exist, but none have been identified as the primary weather stream.
		\retval S_FALSE Unspecified error
		\retval E_OUTOFMEMORY Out of memory
		\retval E_INVALIDARG Unspecified error
		\retval E_FAIL Unspecified error
		\retval ERROR_WEATHER_STREAM_NOT_ASSIGNED No weather streams are attached to this scenario
		\retval E_POINTER Unspecified error
	*/
	virtual NO_THROW HRESULT Simulation_State();
	/** Prepares a scenario's simulation to begin running.  This routine must be called before Simulation_Step(), and must be called to re-run a simulation.  This routine erases any data stored from a previously running simulation.
		\sa ICWFGM_Scenario::Simulation_Reset
		\retval S_OK Scenario ready to execute, but has not begun
		\retval ERROR_GRID_UNINITIALIZED No ICWFGM_GridEngine object has been specified, or that object doesn't have an initialized latitude, longitude, or time zone.
		\retval ERROR_SCENARIO_NO_FIRES No fires have been attached to the scenario
		\retval ERROR_SCENARIO_BAD_TIMES Start or end time is invalid or the internal or output calculations intervals are invalid
		\retval ERROR_SCENARIO_BAD_TIME_STEPS The time steps are invalid
		\retval ERROR_SCENARIO_INVALID_BURNCONDITIONS	Invalid burn condition settings (burning periods between days overlap)
		\retval ERROR_SEVERITY_WARNING The FuelMap for the object implementing the ICWFGM_GridEngine interface has not been set, or no FBP data has been loaded
		\retval ERROR_GRID_WEATHER_NOT_IMPLEMENTED The object implementing the ICWFGM_Grid_Enging interface does not handle weather data, and it was asked to test for valididty against simulation times
		\retval ERROR_SCENARIO_BAD_STATE If this function is run while the scenario is running
		\retval ERROR_GRID_WEATHER_INVALID_DATES Weather grids which were attached are in an invalid state
		\retval S_FALSE Unspecified error
		\retval E_OUTOFMEMORY Insufficient memory
		\retval E_INVALIDARG Unspecified error
		\retval E_FAIL Unspecified error
		\retval ERROR_WEATHER_STREAM_NOT_ASSIGNED No weather streams are attached to this scenario
		\retval E_POINTER Unspecified error
	*/
	virtual NO_THROW HRESULT Simulation_Reset(std::shared_ptr<validation::validation_object> valid, const std::string& name);
	/** Runs the simulation for exactly one calculation time step period of time.  This routine can be called after Simulation_Reset().
		\sa ICWFGM_Scenario::Simulation_Step
		\retval S_OK Scenario ready to execute, but has not begun
		\retval ERROR_SCENARIO_BAD_STATE Scenario is not running
		\retval SUCCESS_SCENARIO_SIMULATION_COMPLETE This step completes simulation
		\retval SUCCESS_SCENARIO_SIMULATION_COMPLETE_EXTENTS The scenario's simulation is complete because the edge of the grid boundary has been met
		\retval SUCCESS_SCENARIO_SIMULATION_COMPLETE_ASSET The scenario's simulation is complete because the correct number of assets have been encountered by the simulated fire(s)
		\retval SUCCESS_SCENARIO_SIMULATION_COMPLETE_STOPCONDITION The scenario's simulation is complete because of 1 or more user defined conditions based on fire growth, saying it's complete
		\retval ERROR_GRID_UNINITIALIZED No ICWFGM_GridEngine object has been specified, or that object doesn't have an initalized latitude, longitude, or time zone
		\retval E_OUTOFMEMORY Insufficient memory
	*/
	virtual NO_THROW HRESULT Simulation_Step();
	/** Clears the current scenario's calculations, unlocks the scenario's objects and clears the displays.
		\sa ICWFGM_Scenario::Simulation_Clear
		\retval S_OK Success
		\retval ERROR_SCENARIO_BAD_STATE Scenario is running
	*/
	virtual NO_THROW HRESULT Simulation_Clear();

	/** Polymorphic.  Retrieves a value for a scenario setting that the client application can use to modify the operation of the scenario, or to determine the state of the simulation.  These values are defined in FireEngine_ext.h.
		\param option Option of interest(a list of possible options appears in "FireEngine_ext.h".  Supported / valid attribute/option index supported are:
		<ul>
		<li><code>CWFGM_SCENARIO_OPTION_TOPOGRAPHY</code>  Boolean.  If true, then the slope component to calculating the WSV output statistic in the FBP standard is set to 0.0.  If true, then the wind speed is used as provided.  This setting also affects the slope component in Dr. Richards' 3D equations.
		<li><code>CWFGM_SCENARIO_OPTION_FMC_TERRAIN</code>  Boolean.  If true, then elevation is used to calculate FMC.  Note that specific rules exist when default elevations have to be used.
		<li><code>CWFGM_SCENARIO_OPTION_WIND</code>  Boolean.  If false, then the wind component to calculating the WSV statistic in the FBP standard is set to 0.0.  If true, then the wind speed is used as provided.
		<li><code>CWFGM_SCENARIO_OPTION_EXTINGUISHMENT</code>  Boolean.  Currently unused.
		<li><code>CWFGM_SCENARIO_OPTION_USE_2DGROWTH</code>  Boolean.  true if using Dr. Richards' 2D equations.  false if using Dr. Richards' 3D equations.
		<li><code>CWFGM_SCENARIO_OPTION_BOUNDARY_STOP</code>  Boolean.  true if the simulation stops when any simulated fire reaches the grid boundary.  false if the simulation should continue (in which case, any perimeter of the fire is stopped and clipped at the grid boundary).
		<li><code>CWFGM_SCENARIO_OPTION_SPOTTING</code>  Boolean.  Currently unused.
		<li><code>CWFGM_SCENARIO_OPTION_BREACHING</code>  Boolean.  Determines whether breaching of vector and gridded fire breaks is allowed.
		<li><code>CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC</code>  Boolean.  Determines whether a dynamic spatial threshold algorithm is used, or if the value specified by CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD is used during simulations.
		<li><code>CWFGM_SCENARIO_OPTION_SINGLETHREADING</code>	Boolean.  true if multithreading within a simulation is disabled.  false if multithreading is enabled.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMPORAL</code> Boolean.  When true, temporal weather interpolation is turned on, for all of the WX and hourly/instantantaneous FWI calculations.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code> Boolean.  When true, spatial weather interpolation is turned on.  This option applies to both WX and FWI values, and will work whether there is 1 or more weather stations assigned to the scenario.  If false, then there should only be one weather stream.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_PRECIP</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code>.  When true, precipitation is spatially interpolated using IDW.  When false, precipitation from the primary weather stream is used.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code>.  When true, wind is spatially interpolated using weighted vector addition rather than the original (described above) approach.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND_VECTOR</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND</code>.  When true, wind is spatially interpolated (WS is defined using IDW, WD is chosen from the closest station); when false, wind from the primary weather stream is used.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMP_RH</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code>.  When true, temperature, dew point temperature, and RH are calculated spatially from adiabatic lapse rates; when false, values from the primary weather stream are used.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI</code> Boolean.  If false, then the current FWI values are returned (possibly interpolated).  If true, then the current FWI values are calculated from the prior FWI values and the current weather values (likely spatially interpolated).
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code> and <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI</code>.  If true, then historical FWI values are calculated to try to attain equilibrium on FWI values.
		<li><code>CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION</code> Boolean.  When true, the precise location of a simulated fire vertex is used when calculating FMC.  When false, the grid's lower left corner is used in calculating FMC.
		<li><code>CWFGM_SCENARIO_OPTION_MULTITHREADING</code>	32-bit unsigned integer.  If multithreading is enabled, then this option determines how many threads to allow.
		<li><code>CWFGM_SCENARIO_OPTION_PERIMETER_RESOLUTION</code> 64-bit floating point.  Maximum distance between any two vertices on a fire perimeter.
		<li><code>CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD</code> 64-bit floating point.  Maximum distance (in grid units) that a vertex is allows to travel while advancing a fire perimeter.
		<li><code>CWFGM_SCENARIO_OPTION_MINIMUM_SPREADING_ROS</code> 64-bit floating point.  Minimum ROS (in m/min, default is 1mm/min) that can support fire growth - less than this and a given vertex will be stopped.
		<li><code>CWFGM_GRID_ATTRIBUTE_LATITUDE</code> 64-bit floating point, radians.
		<li><code>CWFGM_GRID_ATTRIBUTE_LONGITUDE</code> 64-bit floating point, radians.
		<li><code>CWFGM_SCENARIO_OPTION_SPECIFIED_FMC</code> 64-bit floating point.  User-override FMC value for the simulation.  If >= 0, then the value is used, if < 0 then, this value is not used.
		<li><code>CWFGM_SCENARIO_OPTION_DEFAULT_ELEVATION</code> 64-bit floating point, metres.  User specified elevation to use when there is no grid elevation available for at a requested grid location.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITION_SIZE</code> 64-bit floating point.  Diameter (in meters) of the starting circle of an ignition point.  Also used to determine the width of an ignition line.
		<li><code>CWFGM_SCENARIO_OPTION_START_TIME</code>	64-bit unsigned integer.  GMT time provided as seconds since Midnight January 1, 1600
		<li><code>CWFGM_SCENARIO_OPTION_END_TIME</code>		64-bit unsigned integer.  GMT time provided as seconds since Midnight January 1, 1600
		<li><code>CWFGM_SCENARIO_OPTION_CURRENT_TIME</code>	64-bit unsigned integer.  GMT time provided as seconds since Midnight January 1, 1600
		<li><code>CWFGM_SCENARIO_OPTION_TEMPORAL_THRESHOLD_ACCEL</code> 64-bit signed integer.  Units are in seconds.  Maximum time allowed between two adjacent simulation time steps when any ignition is in its acceleration phase (when ROSt is less than 90% of ROSeq).
		<li><code>CWFGM_SCENARIO_OPTION_DISPLAY_INTERVAL</code> 64-bit signed integer.  Units are in seconds.  Time interval for output fire perimeters, or 0 if every time step is to be outputted.
		<li><code>CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE</code> Boolean.  When true, non-displayable time steps will not be retained, to save on memory overhead.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_DX</code> 64-bit floating point.  Units are in meters. How much to nudge ignitions to perform probabilistic analyses on ignition location. Primarily used when ignition information is not 100% reliable.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_DY</code> 64-bit floating point.  Units are in meters. How much to nudge ignitions to perform probabilistic analyses on ignition location. Primarily used when ignition information is not 100% reliable.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_DT</code> 64-bit signed integer.  Units are in seconds. How much to nudge ignitions to perform probabilistic analyses on ignition location and start time. Primarily used when ignition information is not 100% reliable.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_DWD</code> 64-bit floating point.  Units are in compass degrees. How much to nudge wind direction to perform probabilistic analyses on weather. Applied after all patches and grids, and does not recalculate any FWI calculations. Applied before any FBP calculations. Provided in compass degrees, -360 to 360 is acceptable. Applied to both simulations, and to instantaneous calculations as shown on the map trace view query, for consistency. Primarily used when weather information does not have the expected fidelity.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_OWD</code> 64-bit floating point.  Units are in compass degrees. Override value for wind direction to perform probabilistic analyses on weather. Applied after all patches and grids, and does not recalculate any FWI calculations. Applied before any FBP calculations. Provided in compass degrees, 0 to 360 is acceptable, or -1 to disable. Applied to both simulations, and to instantaneous calculations as shown on the map trace view query, for consistency. Primarily used when weather information does not have the expected fidelity.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE</code> 64-bit floating point.  0 to 100 exclusive. Growth percentile, to apply to specific fuel types.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE_ENABLE</code> Boolean. Determines whether CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE is active.
		<li><code>CWFGM_GRID_ATTRIBUTE_TIMEZONE_ID</code>	32-bit unsigned integer. An ID for a predefined set of timezone details. Can be used with a WorldLocations SetTimeZoneOffset method to set the timezone properties.
		<li><code>CWFGM_GRID_ATTRIBUTE_TIMEZONE</code>		64-bit signed integer.  Units are in seconds, relative to GMT.  For example, MST (Mountain Standard Time) would be -6 * 60 * 60 seconds.  Valid values are from -12 hours to +12 hours.
		<li><code>CWFGM_GRID_ATTRIBUTE_DAYLIGHT_SAVINGS</code>	64-bit signed integer.  Units are in seconds.  Amount of correction to apply for daylight savings time.
		<li><code>CWFGM_GRID_ATTRIBUTE_DST_START</code>	64-bit unsigned integer.  Units are in seconds.  Julian date determining when daylight savings starts within the calendar year.
		<li><code>CWFGM_GRID_ATTRIBUTE_DST_END</code>	64-bit unsigned integer.  Units are in seconds.  Julian date determining when daylight savings ends within the calendar year.
		<li><code>CWFGM_ATTRIBUTE_LOAD_WARNING</code>	BSTR.  Any warnings generated by the COM object when deserializating.
		<li><code>CWFGM_SCENARIO_OPTION_CACHE_GRID_POINTS</code> Boolean.  When true, caches will be calculated to determine the closest points (for statistics purposes) to the center of each grid cell.
		<li><code>CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT</code> Boolean.  When TRUE, then we limit the sine curve to '1' when considering adding points to a convex portion of the hull
		<li><code>CWFGM_SCENARIO_OPTION_GRID_DECIMATION</code> 64-bit floating point.  If 0, then all accuracy is retained in calculations.  If not 0, then specifies the grid resolution to pull all values to
		<li><code>CWFGM_SCENARIO_OPTION_FALSE_ORIGIN</code>	Boolean. Whether or not to apply the grid's (original) false origin to FireEngine calc's
		<li><code>CWFGM_SCENARIO_OPTION_FALSE_SCALING</code> Boolean. Whether or not to apply the grid's fuel scaling to FireEngine calc's
		</ul>
		\param value Polymorphic.  Value retrieved.
		\sa ICWFGM_Scenario::GetAttribute
		
		\retval E_POINTER The address provided for value is invalid
		\retval S_OK Successful
		\retval E_INVALIDARG An unknown option was asked for
		\retval ERROR_SCENARIO_OPTION_INVALID An invalid option has been selected
	*/
	virtual NO_THROW HRESULT GetAttribute(std::uint16_t option, PolymorphicAttribute *value) const;
	/** Polymorphic.  Sets a value for a scenario setting that the client application can modify to change the operation of the scenario.  
		\param option Option of interest(a list of possible options appears in "FireEngine_ext.h".  Supported / valid attribute/option index supported are:
		<ul>
		<li><code>CWFGM_SCENARIO_OPTION_TOPOGRAPHY</code>  Boolean.  If true, then the slope component to calculating the WSV output statistic in the FBP standard is set to 0.0.  If true, then the wind speed is used as provided.  This setting also affects the slope component in Dr. Richards' 3D equations.
		<li><code>CWFGM_SCENARIO_OPTION_FMC_TERRAIN</code>  Boolean.  If true, then elevation is used to calculate FMC.  Note that specific rules exist when default elevations have to be used.
		<li><code>CWFGM_SCENARIO_OPTION_WIND</code>  Boolean.  If false, then the wind component to calculating the WSV statistic in the FBP standard is set to 0.0.  If true, then the wind speed is used as provided.
		<li><code>CWFGM_SCENARIO_OPTION_EXTINGUISHMENT</code>  Boolean.  Currently unused.
		<li><code>CWFGM_SCENARIO_OPTION_USE_2DGROWTH</code>  Boolean.  true if using Dr. Richards' 2D equations.  false if using Dr. Richards' 3D equations.
		<li><code>CWFGM_SCENARIO_OPTION_BOUNDARY_STOP</code>  Boolean.  true if the simulation stops when any simulated fire reaches the grid boundary.  false if the simulation should continue (in which case, any perimeter of the fire is stopped and clipped at the grid boundary).
		<li><code>CWFGM_SCENARIO_OPTION_SPOTTING</code>  Boolean.  Currently unused.
		<li><code>CWFGM_SCENARIO_OPTION_BREACHING</code>  Boolean.  Determines whether breaching of vector and gridded fire breaks is allowed.
		<li><code>CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD_DYNAMIC</code>  Boolean.  Determines whether a dynamic spatial threshold algorithm is used, or if the value specified by CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD is used during simulations.
		<li><code>CWFGM_SCENARIO_OPTION_SINGLETHREADING</code>	Boolean.  true if multithreading within a simulation is disabled.  false if multithreading is enabled.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMPORAL</code> Boolean.  When true, temporal weather interpolation is turned on, for all of the WX and hourly/instantantaneous FWI calculations.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code> Boolean.  When true, spatial weather interpolation is turned on.  This option applies to both WX and FWI values, and will work whether there is 1 or more weather stations assigned to the scenario.  If false, then there should only be one weather stream.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_PRECIP</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code>.  When true, precipitation is spatially interpolated using IDW.  When false, precipitation from the primary weather stream is used.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code>.  When true, wind is spatially interpolated (WS is defined using IDW, WD is chosen from the closest station); when false, wind from the primary weather stream is used.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND_VECTOR</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_WIND</code>.  When true, wind is spatially interpolated (WS is defined using IDW, WD is chosen from the closest station); when false, wind from the primary weather stream is used.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_TEMP_RH</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code>.  When true, temperature, dew point temperature, and RH are calculated spatially from adiabatic lapse rates; when false, values from the primary weather stream are used.
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI</code> Boolean.  If false, then the current FWI values are returned (possibly interpolated).  If true, then the current FWI values are calculated from the prior FWI values and the current weather values (likely spatially interpolated).
		<li><code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_HISTORY</code> Boolean.  Conditional on <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_SPATIAL</code> and <code>CWFGM_SCENARIO_OPTION_WEATHER_INTERPOLATE_CALCFWI</code>.  If true, then historical FWI values are calculated to try to attain equilibrium on FWI values.
		<li><code>CWFGM_SCENARIO_OPTION_ACCURATE_FMC_LOCATION</code> Boolean.  When true, the precise location of a simulated fire vertex is used when calculating FMC.  When false, the grid's lower left corner is used in calculating FMC.
		<li><code>CWFGM_SCENARIO_OPTION_MULTITHREADING</code>	32-bit unsigned integer.  If multithreading is enabled, then this option determines how many threads to allow.
		<li><code>CWFGM_SCENARIO_OPTION_PERIMETER_RESOLUTION</code> 64-bit floating point.  Maximum distance between any two vertices on a fire perimeter.
		<li><code>CWFGM_SCENARIO_OPTION_SPATIAL_THRESHOLD</code> 64-bit floating point.  Maximum distance (in grid units) that a vertex is allows to travel while advancing a fire perimeter.
		<li><code>CWFGM_SCENARIO_OPTION_MINIMUM_SPREADING_ROS</code> 64-bit floating point.  Minimum ROS (in m/min, default is 1mm/min) that can support fire growth - less than this and a given vertex will be stopped.
		<li><code>CWFGM_GRID_ATTRIBUTE_LATITUDE</code> 64-bit floating point, radians.
		<li><code>CWFGM_GRID_ATTRIBUTE_LONGITUDE</code> 64-bit floating point, radians.
		<li><code>CWFGM_SCENARIO_OPTION_SPECIFIED_FMC</code> 64-bit floating point.  User-override FMC value for the simulation.  If >= 0, then the value is used, if < 0 then, this value is not used.
		<li><code>CWFGM_SCENARIO_OPTION_DEFAULT_ELEVATION</code> 64-bit floating point, metres.  User specified elevation to use when there is no grid elevation available for at a requested grid location.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITION_SIZE</code> 64-bit floating point.  Diameter (in meters) of the starting circle of an ignition point.  Also used to determine the width of an ignition line.
		<li><code>CWFGM_SCENARIO_OPTION_START_TIME</code>	64-bit unsigned integer.  GMT time provided as seconds since Midnight January 1, 1600
		<li><code>CWFGM_SCENARIO_OPTION_END_TIME</code>		64-bit unsigned integer.  GMT time provided as seconds since Midnight January 1, 1600
		<li><code>CWFGM_SCENARIO_OPTION_TEMPORAL_THRESHOLD_ACCEL</code> 64-bit signed integer.  Units are in seconds.  Maximum time allowed between two adjacent simulation time steps when any ignition is in its acceleration phase (when ROSt is less than 90% of ROSeq).
		<li><code>CWFGM_SCENARIO_OPTION_DISPLAY_INTERVAL</code> 64-bit signed integer.  Units are in seconds.  Time interval for output fire perimeters, or 0 if every time step is to be outputted.
		<li><code>CWFGM_SCENARIO_OPTION_PURGE_NONDISPLAYABLE</code> Boolean.  When true, non-displayable time steps will not be retained, to save on memory overhead.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_DX</code> 64-bit floating point.  Units are in meters. How much to nudge ignitions to perform probabilistic analyses on ignition location. Primarily used when ignition information is not 100% reliable.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_DY</code> 64-bit floating point.  Units are in meters. How much to nudge ignitions to perform probabilistic analyses on ignition location. Primarily used when ignition information is not 100% reliable.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_DT</code> 64-bit signed integer.  Units are in seconds. How much to nudge ignitions to perform probabilistic analyses on ignition location and start time. Primarily used when ignition information is not 100% reliable.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_DWD</code> 64-bit floating point.  Units are in compass degrees. How much to nudge wind direction to perform probabilistic analyses on weather. Applied after all patches and grids, and does not recalculate any FWI calculations. Applied before any FBP calculations. Provided in compass degrees, -360 to 360 is acceptable. Applied to both simulations, and to instantaneous calculations as shown on the map trace view query, for consistency. Primarily used when weather information does not have the expected fidelity.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_OWD</code> 64-bit floating point.  Units are in compass degrees. Override value for wind direction to perform probabilistic analyses on weather. Applied after all patches and grids, and does not recalculate any FWI calculations. Applied before any FBP calculations. Provided in compass degrees, 0 to 360 is acceptable, or -1 to disable. Applied to both simulations, and to instantaneous calculations as shown on the map trace view query, for consistency. Primarily used when weather information does not have the expected fidelity.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE</code> 64-bit floating point.  0 to 100 exclusive. Growth percentile, to apply to specific fuel types.
		<li><code>CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE_ENABLE</code> Boolean. Determines whether CWFGM_SCENARIO_OPTION_IGNITIONS_PERCENTILE is active.
		<li><code>CWFGM_GRID_ATTRIBUTE_TIMEZONE_ID</code>	32-bit unsigned integer. An ID for a predefined set of timezone details. Can be used with a WorldLocations SetTimeZoneOffset method to set the timezone properties.
		<li><code>CWFGM_GRID_ATTRIBUTE_TIMEZONE</code>		64-bit signed integer.  Units are in seconds, relative to GMT.  For example, MST (Mountain Standard Time) would be -6 * 60 * 60 seconds.  Valid values are from -12 hours to +12 hours.
		<li><code>CWFGM_GRID_ATTRIBUTE_DAYLIGHT_SAVINGS</code>	64-bit signed integer.  Units are in seconds.  Amount of correction to apply for daylight savings time.
		<li><code>CWFGM_GRID_ATTRIBUTE_DST_START</code>	64-bit unsigned integer.  Units are in seconds.  Julian date determining when daylight savings starts within the calendar year.
		<li><code>CWFGM_GRID_ATTRIBUTE_DST_END</code>	64-bit unsigned integer.  Units are in seconds.  Julian date determining when daylight savings ends within the calendar year.
		<li><code>CWFGM_SCENARIO_OPTION_CACHE_GRID_POINTS</code> Boolean.  When true, caches will be calculated to determine the closest points (for statistics purposes) to the center of each grid cell.
		<li><code>CWFGM_SCENARIO_OPTION_SUPPRESS_TIGHT_CONCAVE_ADDPOINT</code> Boolean.  When TRUE, then we limit the sine curve to '1' when considering adding points to a convex portion of the hull
		<li><code>CWFGM_SCENARIO_OPTION_GRID_DECIMATION</code> 64-bit floating point.  If 0, then all accuracy is retained in calculations.  If not 0, then specifies the grid resolution to pull all values to
		<li><code>CWFGM_SCENARIO_OPTION_FALSE_ORIGIN</code>	Boolean. Whether or not to apply the grid's (original) false origin to FireEngine calc's
		<li><code>CWFGM_SCENARIO_OPTION_FALSE_SCALING</code> Boolean. Whether or not to apply the grid's fuel scaling to FireEngine calc's
		</ul>
		\param value  Polymorphic.  Value for option being set.
		\sa ICWFGM_Scenario::SetAttribute

		\retval S_OK Successful
		\retval ERR0R_SCENARIO_OPTION_INVALID An unknown boolean option was asked for
		\retval ERROR_SCENARIO_BAD_STATE If this function is run while the scenario is running
		\retval E_INVALIDARG If value is not 0 or 1
		\retval E_FAIL Unspecified error
		\retval E_POINTER The address provided is invalid
	*/
	virtual NO_THROW HRESULT SetAttribute(std::uint16_t option, const PolymorphicAttribute &value);

public:
	/** Retrieves the rectangular area encompassing all burned areas of all fires at a specific time during the simulation. 
		\param time A GMT time provided as seconds since Midnight January 1, 1600
		\param box: May be NULL. If provided, then the rectangle is filled or adjusted to indicate the maximum size of the cell containing the requested point which is constant and uniform in content and value.
		\sa ICWFGM_Scenario::GetBurnedBox

		\retval E_POINTER The address provided for left, right, bottom, or top is invalid
		\retval S_OK Successful
		\retval ERROR_SCENARIO_BAD_STATE If the function is run while a scenario is running
		\retval ERROR_SEVERITY_WARNING Unspecified error
		\retval SUCCESS_FIRE_NOT_STARTED Either no fires or the request predates the start time of the simulation
	*/
	virtual NO_THROW HRESULT GetBurnedBox(const HSS_Time::WTime &time, XY_Rectangle *box) const;
	/** Retrieves whether the location at (X, Y) was burned before or during time.
		\param X X coordinate
		\param Y Y coordinate
		\param time A GMT time provided as seconds since Midnight January 1, 1600
		\param burned If the location has been burned, this value is set to 1, otherwise it is set to 0.
		\sa ICWFGM_Scenario::IsXYBurned

		\retval E_POINTER The address provided for burned is invalid
		\retval S_OK Successful
		\retval ERROR_SCENARIO_BAD_STATE If the function is run while a scenario is running
		\retval ERROR_NODATA|ERROR_SEVERITY WARNING Nothing initialized yet
		\retval SUCCESS_FINE_NOT_STARTED Either no fires or the request predates the start time of the simulation
	*/
	virtual NO_THROW HRESULT IsXYBurned(const XY_Point &pt, const HSS_Time::WTime &time, bool *burned) const;
	/** Removes one displayable step during a simulation.
		\sa ICWFGM_Scenario::Simulation_StepBack
		\retval S_OK Successful
		\retval ERROR_SCENARIO_BAD_STATE Scenario is not running
		\retval ERR0R_GRID_UNINTIALIZED No ICWFGM_GridEngine object has been specified, or that object doesn't have an initialized latitude, longitude, or time zone.
	*/
	virtual NO_THROW HRESULT Simulation_StepBack();

	/** Given a scenario, this method returns the number of output time steps that have been calculated in the simulation for this fire.  A fire can simultaneously contain statistical information for a variety of scenarios.
		\param steps Number of output times steps
		\sa ICWFGM_Scenario::GetNumberSteps

		\retval E_POINTER The address provided for steps is invalid
		\retval S_OK Successful
		\retval ERROR_FIRE_SCENARIO_UNKNOWN This fire object is not attached to the scenario specified by the scenario parameter
		\retval ERROR_SCENARIO_BAD_STATE If the function is run while a scenario is running
	*/
	virtual NO_THROW HRESULT GetNumberSteps(std::uint32_t *steps) const;
	/** Given an array, return the times for each time step calculated in the simulation.  size is passed in as the size of the array, and returned as the actual number of elements set.  times is a pointer to an array that will receive the GMT times for each calculated time step.  The size of the array that is required can be obtained by calling GetNumberSteps().
		\param size Size of the times array
		\param times Array of times
		\sa ICWFGM_Scenario::GetStepsArray

		\retval E_POINTER The address provided for size, or times is invalid
		\retval S_OK Successful
		\retval E_OUTOFMEMORY Insufficient memory
	*/
	virtual NO_THROW HRESULT GetStepsArray(std::uint32_t *size, std::vector<HSS_Time::WTime> *times) const;
	/** Returns the number of fires (burning, partially burning, or not burning) which are in the simulation at the specified time.
		\param time Time for the query
		\param count Return value, number of fires
		\sa ICWFGM_Scenario::GetNumberFires

		\retval E_POINTER The address provided for time, or size is invalid
		\retval S_OK Successful
		\retval ERROR_NO_DATA|ERROR_SEVERITY_WARNING Nothing initialized yet
		\retval ERROR_SCENARIO_BAD_STATE If the function is run while a scenario is not running
		\retval SUCCESS_FIRE_NOT_STARTED Either no fires or the request predates the start time of the simulation
	*/
	virtual NO_THROW HRESULT GetNumberFires(HSS_Time::WTime *time, std::uint32_t *count) const;
	/** Given a fire at a specific time in the simulation, identifies the ignition from which the fire originated.
		\param index Index of the fire
		\param time Time of the simulation
		\param fire Return value; ignition
		\sa ICWFGM_Scenario::IgnitionAtFireIndex

		\retval E_POINTER The address provided for fire is invalid
		\retval S_OK Successful
		\retval ERROR_NODATA|ERROR_SEVERITY_WARNING Nothing initialized yet
		\retval ERROR_SCENARIO_BAD_STATE If the function is run while a scenario is not running
		\retval SUCCESS_FIRE_NOT_STARTED Either no fires or  the request predates the start time of the simulation
		\retval ERROR_SCENARIO_FIRE_UNKNOWN This fire object is not attached to the scenario specified by the scenario parameter
	*/
	virtual NO_THROW HRESULT IgnitionAtFireIndex(std::uint32_t index, HSS_Time::WTime *time, boost::intrusive_ptr<CCWFGM_Ignition> *fire) const;
	/** Given a fire, this method returns the number of points defining a fire front at the time specified in the simulation.  time is passed in as a requested time and returned as the actual time that the vector size relates to.
		\param fire Index of the fire
		\param time Specified time(on return, time is set to the actual time the vector size relates to)
		\param size Number of points defining a fire front
		\sa ICWFGM_Scenario::GetVectorSize

		\retval E_POINTER The address provided for time or size is invalid
		\retval S_OK Successful(if time is invalid)
		\retval ERROR_FIRE_SCENARIO_UNKOWN This fire object is not attached to the scenario specified by the scenario parameter
		\retval SUCCESS_FIRE_NOT_STARTED The scenario's simulation has been reset to run, but hasn't actually stepped yet(in this case size is returned to 0)
		\retval ERROR_NO_DATA|ERROR|SEVERITY|WARNING Nothing has been initialized yet
		\retval ERROR_SCENARIO_BAD_STATE If the function is run without any ignitions assigned to the scenario
	*/
	virtual NO_THROW HRESULT GetVectorSize(std::uint32_t fire, HSS_Time::WTime *time, std::uint32_t *size) const;
	/** Given a fire, this method returns the points defining a fire front at the time specified in the simulation.  size is passed in as the size of the array, and returned as the actual number of elements set.  Locations are returned in X/Y pairs in the appropriate array (in grid units) of the fire.  time is passed in as a requested time and returned as the actual time that the data is for.
		\param fire Index of the fire
		\param time Specified time(on return, time is set to the actual time the array of coordinates relates to)
		\param size Size of the xy_pairs array
		\param xy_pairs 2D Array of coordinates
		\sa ICWFGM_Scenario::GetVectorArray

		\retval E_POINTER The address provided for time, size or xy_pairs is invalid
		\retval S_OK Successful
		\retval E_OUTOFMEMORY Insufficient memory 
		\retval ERROR_FIRE_SCENARIO_UNKOWN This fire object is not attached to the scenario specified by the scenario
		\retval SUCCESS_FIRE_NOT_STARTED The scenario's simulation has been reset to turn, but hasn't actually stepped yet(in this case, size is returned 0)
		\retval SUCCESS_FIRE_BURNED_OUT This fire has completely burned out in this simulation and has no remaining points defining it front for this time (size is returned 0)
		\retval ERROR_NO_DATA|ERROR_SEVERITY_WARNING Nothing has been initialized yet
		\retval ERROR_SCENARIO_BAD_STATE If the function is run without any ignition assigned to the scenario
	*/
	virtual NO_THROW HRESULT GetVectorArray(std::uint32_t fire, HSS_Time::WTime *time, std::uint32_t *size, XY_Poly *xy_pairs) const;

	/** Given a fire, this method returns a statistic for each point defining a fire front at the time specified in the simulation.  stat must be a valid statistic, as defined in FireEngine_ext.h.  size is passed in as the size of the array, and returned as the actual number of elements set.  xand y are pointers to arrays that will receive the locations (in grid units) of the fire. time is passed in as a requested time and returned as the actual time that the data is for.  Statistics data is returned for both active and inactive points.
		\param fire Index of the fire
		\param time Specified GMT time since Midnight January 1, 1600
		\param stat Requested statistic, valid statistics are:
			<ul>
			<li><code>CWFGM_FIRE_STAT_FBP_RSI</code> initial spread rate without BUI effect, from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_FBP_ROSEQ</code> equilibrium rate of spread from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_FBP_ROS</code> rate of spread from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_FBP_BROS</code> back rate of spread from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_FBP_FROS</code> flank rate of spread from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_RAZ</code> wind speed vector azimuth from FBP calculations (radians, Cartesian)
			<li><code>CWFGM_FIRE_STAT_ROS</code> rate of spread calculated from Dr. Richards' ellipse equations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_CFB</code> crown fraction burned (unitless), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_CFC</code> crown fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_SFC</code> surface fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_TFC</code> total fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_FI</code> fire intensity, based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_FLAMELENGTH</code> flame length (metres), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_OUTER</code> Boolean.  (identifies if the point is on the outer hull) 
			</ul>
		\param size Size of the stats array
		\param stats Array of statistics
		\sa ICWFGM_Scenario::GetStatsArray

		\retval E_POINTER The address provided for time, size or stats is invalid
		\retval S_OK Successful 
		\retval E_OUTOFMEMORY Insufficient memory 
		\retval ERROR_NOT_DATA|ERROR_SEVERITY_WARNING Nothing has been initialized yet
		\retval SUCCESS_FIRE_NOT_STARTED The scenario's simulation has been reset to run but hasn't actually stepped yet(in this case, size is returned 0)
		\retval ERROR_FIRE_SCENARIO_UNKNOWN This fire object is not attached to the scenario specified by the scenario parameter
		\retval ERROR_SCENARIO_BAD_STATE If the function is run without any ignitions assigned to the scenario.
	*/
	virtual NO_THROW HRESULT GetStatsArray(std::uint32_t fire, HSS_Time::WTime *time, std::uint16_t stat, std::uint32_t *size, std::vector<double> *stats) const;
	/** This method returns a particular statistic for the fire, for a specific simulation at a specific time.  stat must be a valid statistic, as defined in FireEngine_ext.h. time is passed in as a requested time and returned as the actual time that the data is for.

		\param fire Index of the fire
		\param time Specified GMT time since Midnight January 1, 1600
		\param stat Requested statistic.  Valid statistics for query are:
		<ul>
			<li><code>CWFGM_FIRE_STAT_TOTAL_PERIMETER</code> 64-bit floating point.  (metres)  Total fire perimeter, including interior and exterior and active/inactive portions.
			<li><code>CWFGM_FIRE_STAT_EXTERIOR_PERIMETER</code> 64-bit floating point.  (metres)  Total exterior fire perimeter, including active and inactive portions.
			<li><code>CWFGM_FIRE_STAT_ACTIVE_PERIMETER</code> 64-bit floating point.  (metres)  Portion of the fire front considered active (interior and exterior) (where 1 or both vertices are active)
			<li><code>CWFGM_FIRE_STAT_AREA</code> 64bit floating point. (sq. metres)  Total area of the fire.
			<li><code>CWFGM_FIRE_STAT_TOTAL_PERIMETER_CHANGE</code> 64-bit floating point.  (metres)  Change in the total perimeter growth.
			<li><code>CWFGM_FIRE_STAT_TOTAL_PERIMETER_GROWTH</code> 64-bit floating point.  (metres per minute)  Rate of change in the total perimeter growth rate.
			<li><code>CWFGM_FIRE_STAT_EXTERIOR_PERIMETER_CHANGE</code> 64-bit floating point.  (metres)  Change in the exterior perimeter growth.
			<li><code>CWFGM_FIRE_STAT_EXTERIOR_PERIMETER_GROWTH</code> 64-bit floating point.  (metres per minute)  Rate of change in the exterior perimeter growth rate.
			<li><code>CWFGM_FIRE_STAT_ACTIVE_PERIMETER_CHANGE</code> 64-bit floating point.  (metres)  Change in the active perimeter growth.
			<li><code>CWFGM_FIRE_STAT_ACTIVE_PERIMETER_GROWTH</code> 64-bit floating point.  (metres per minute)  Rate of change in the active perimeter growth rate.
			<li><code>CWFGM_FIRE_STAT_AREA_CHANGE</code> 64-bit floating point.  (sq. metres).  Change in fire area.
			<li><code>CWFGM_FIRE_STAT_AREA_GROWTH</code> 64-bit floating point.  (sq. metres).  Rate of change in the fire area.
			<li><code>CWFGM_FIRE_STAT_NUM_POINTS</code> 32-bit integer.  Number of vertices defining the fire perimeter(s).
			<li><code>CWFGM_FIRE_STAT_NUM_ACTIVE_POINTS</code> 32-bit integer.  Number of active vertices defining the fire perimeter(s).
			<li><code>CWFGM_FIRE_STAT_CUMULATIVE_NUM_ACTIVE_POINTS</code> 32-bit integer.  Cumulative number of active vertices defining the fire perimeter(s).
			<li><code>CWFGM_FIRE_STAT_CUMULATIVE_NUM_POINTS</code> 32-bit integer.  Total, cumulative number of verticies defining the simulation's perimeters.
			<li><code>CWFGM_FIRE_STAT_NUM_FRONTS</code> 32-bit integer.  Number of fire fronts (interior and exterior).
			<li><code>CWFGM_FIRE_STAT_NUM_ACTIVE_FRONTS</code> 32-bit integer.  Number of fire fronts (interior and exterior) which have at least 1 active vertex.
			<li><code>CWFGM_FIRE_STAT_ROS</code> 64-bit floating point. Maximum rate of spread calculated from Dr. Richards' ellipse equations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_CFB</code> 64-bit floating point. Maximum crown fraction burned (unitless), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_CFC</code> 64-bit floating point. Maximum crown fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_SFC</code> 64-bit floating point. Maximum surface fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_TFC</code> 64-bit floating point. Maximum total fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_FI</code> 64-bit floating point. Maximum fire intensity, based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_FLAMELENGTH</code> 64-bit floating point. Maximum flame length (metres), based on ROS from Dr. Richards' ellipse equations
		<li><code>CWFGM_FIRE_STAT_TIMESTEP_REALTIME</code> 64-bit integer.  Number of real-time (clock) seconds to calculate the current display time step
		<li><code>CWFGM_FIRE_STAT_TIMESTEP_CUMULATIVE_REALTIME</code> 64-bit integer.  Number of real-time (clock) seconds to calculate all display time steps
		<li><code>CWFGM_FIRE_STAT_TIMESTEP_CUMULATIVE_BURNING_SECS</code> 64-bit integer.  Number of simulated seconds that burning was allowed since the start of the simulation
		</ul>
		\param stats Calculated statistic value
		\sa ICWFGM_Scenario::GetStats

		\retval E_POINTER The address provided for time or stats is invalid
		\retval S_OK Successful
		\retval ERROR_NO_DATA|ERROR_SEVERITY_WARNING Nothing has been initialized yet
		\retval ERROR_FIRE_INVALID_TIME If the time is invalid
		\retval ERROR_FIRE_SCENARIO_UNKNOWN This fire object is not attached to the scenario specified by the scenario parameter
		\retval SUCCESS_FIRE_NOT_STARTED Fire not yet started (stats is set to 0)
		\retval ERROR_FIRE_STAT_UNKNOWN If the stat does not resolve to a known statistic
	*/
	virtual NO_THROW HRESULT GetStats(std::uint32_t fire, ICWFGM_Fuel *fuel, HSS_Time::WTime *time, std::uint16_t stat, std::uint16_t discretization, PolymorphicAttribute *stats) const;
	/** This method returns a particular statistic for a specific location in the fire/grid, for a specific simulation at a specific time.  stat must be a valid statistic, as defined in FireEngine_ext.h. time is passed in as a requested time and returned as the actual time that the data is for.
		\param pt The coordinate
		\param time Specified GMT time since January 1, 1600
		\param stat Request statistic.  Valid statistics for query are:
		<ul>
		<li><code>CWFGM_FIRE_STAT_FBP_RSI</code> initial spread rate without BUI effect, from FBP calculations (metres per minute)
		<li><code>CWFGM_FIRE_STAT_FBP_ROSEQ</code> equilibrium rate of spread from FBP calculations (metres per minute)
		<li><code>CWFGM_FIRE_STAT_FBP_ROS</code> rate of spread from FBP calculations (metres per minute)
		<li><code>CWFGM_FIRE_STAT_FBP_BROS</code> back rate of spread from FBP calculations (metres per minute)
		<li><code>CWFGM_FIRE_STAT_FBP_FROS</code> flank rate of spread from FBP calculations (metres per minute)
		<li><code>CWFGM_FIRE_STAT_RAZ</code> wind speed vector azimuth from FBP calculations (radians, Cartesian)
		<li><code>CWFGM_FIRE_STAT_CFB</code> crown fraction burned (unitless), based on ROS from Dr. Richards' ellipse equations
		<li><code>CWFGM_FIRE_STAT_CFC</code> crown fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
		<li><code>CWFGM_FIRE_STAT_SFC</code> surface fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
		<li><code>CWFGM_FIRE_STAT_TFC</code> total fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
		<li><code>CWFGM_FIRE_STAT_FI</code> fire intensity, based on ROS from Dr. Richards' ellipse equations
		<li><code>CWFGM_FIRE_STAT_FLAMELENGTH</code> flame length (metres), based on ROS from Dr. Richards' ellipse equations
		<li><code>CWFGM_FIRE_STAT_FMC</code> foliar moisture content
		<li><code>FUELCOM_ATTRIBUTE_CURINGDEGREE</code> grass curing degree if grass
		<li><code>CWFGM_SCENARIO_OPTION_GREENUP</code> green-up state
		<li><code>FUELCOM_ATTRIBUTE_PC</code> PC if mixed fuel type
		<li><code>FUELCOM_ATTRIBUTE_PDF</code> PDF if dead-mixed fuel type
		<li><code>FUELCOM_ATTRIBUTE_CBH</code> crown base height
		<li><code>FUELCOM_ATTRIBUTE_TREE_HEIGHT</code> tree height
		<li><code>FUELCOM_ATTRIBUTE_FUELLOAD</code> grass fuel load if grass
		<li><code>FUELCOM_ATTRIBUTE_CFL</code> CFL, crown fuel load
		<li><code>CWFGM_SCENARIO_OPTION_GRASSPHENOLOGY</code> grass phenology state, if grass
		<li><code>CWFGM_FIRE_STAT_ROS</code> rate of spread calculated from Dr. Richards' ellipse equations (metres per minute)
		<li><code>CWFGM_FIRE_STAT_ACTIVE</code> Boolean.  (Identifies if the point is active or stopped.)
		</ul>
		\param interp_method How to calculate/determine the value of the statistic requested
		<ul>
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_CLOSEST_VERTEX</code> Picks the closest vertex from 1 of 2 specific fire perimeters: the one which first contained the point of interest, and the perimeter immediately prior to that.
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_IDW</code> Calculates the statistic using IDW, using all local, active vertices.
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_AREA_WEIGHTING</code> Calculates the statistic using weighting based on union between voronoi regions and the grid cell of interest.
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_VORONOI_OVERLAP</code> Calculates the statistic using weighting based on the union of the point's inserted voronoi region and the voronoi regions of each vertex without the point.
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_CALCULATE</code> Calculates the statistic in the same manner as the simulation would.
		</ul>
		\param stats Calculated statistic value
		\sa ICWFGM_Scenario::GetXYStats

		\retval E_POINTER The address provided for time or stats is invalid
		\retval S_OK Successful
		\retval ERROR_NO_DATA|ERROR_SEVERITY_WARNING Nothing has been initialized yet
		\retval ERROR_FIRE_INVALID_TIME If the time is invalid
		\retval ERROR_FIRE_SCENARIO_UNKNOWN This fire object is not attached to the scenario specified by the scenario parameter
		\retval SUCCESS_FIRE_NOT_STARTED Fire not yet started(stats is set to 0)
		\retval ERROR_FIRE_STAT_UNKNOWN If stat does not rewolve to a known statistic
		\retval ERROR_SCENARIO_BAD_STATE If the function is run without a running scenario
		\retval E_OUTOFMEMORY Insufficient memory
	*/
	virtual NO_THROW HRESULT GetXYStats(const XY_Point &min_pt, const XY_Point &max_pt, XYStatOptions* options, std::uint16_t stat, NumericVariant *stats);
	/** This method returns a particular statistic for a specific location in the fire/grid, for a specific simulation at a specific time.  stat must be a valid statistic, as defined in FireEngine_ext.h. time is passed in as a requested time and returned as the actual time that the data is for.
		\param X The x coordinate
		\param Y The y coordinate
		\param time Specified GMT time since January 1, 1600
		\param interp_method How to calculate/determine the value of the statistic requested
		<ul>
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_CLOSEST_VERTEX</code> Picks the closest vertex from 1 of 2 specific fire perimeters: the one which first contained the point of interest, and the perimeter immediately prior to that.
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_IDW</code> Calculates the statistic using IDW, using all local, active vertices.
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_AREA_WEIGHTING</code> Calculates the statistic using weighting based on union between voronoi regions and the grid cell of interest.
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_VORONOI_OVERLAP</code> Calculates the statistic using weighting based on the union of the point's inserted voronoi region and the voronoi regions of each vertex without the point.
		<li><code>SCENARIO_XYSTAT_TECHNIQUE_CALCULATE</code> Calculates the statistic in the same manner as the simulation would.
		</ul>
		\param stats Array of stat's to fill in, check GetXYStats for acceptable values.

		\retval E_POINTER The address provided for time or stats is invalid
		\retval S_OK Successful
		\retval ERROR_NO_DATA|ERROR_SEVERITY_WARNING Nothing has been initialized yet
		\retval ERROR_FIRE_INVALID_TIME If the time is invalid
		\retval ERROR_FIRE_SCENARIO_UNKNOWN This fire object is not attached to the scenario specified by the scenario parameter
		\retval SUCCESS_FIRE_NOT_STARTED Fire not yet started(stats is set to 0)
		\retval ERROR_FIRE_STAT_UNKNOWN If stat does not rewolve to a known statistic
		\retval ERROR_SCENARIO_BAD_STATE If the function is run without a running scenario
		\retval E_OUTOFMEMORY Insufficient memory
	*/
	virtual NO_THROW HRESULT GetXYStatsSet(const XY_Point& min_pt, const XY_Point& max_pt, XYStatOptions *options, std::vector<XYStat> *stats);
	/** This method returns a count of occurence particular statistic for a specific range of values, given a fire and time.  stat must be a valid statistic, as defined in FireEngine.h 
		\param fire Index of the fire
		\param time Specified GMT time since Midnight January 1, 1600
			\param stat Requested statistic.  Valid statistics for query are:
			<ul>
			<li><code>CWFGM_FIRE_STAT_FBP_RSI</code> initial spread rate without BUI effect, from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_FBP_ROSEQ</code> equilibrium rate of spread from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_FBP_ROS</code> rate of spread from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_FBP_BROS</code> back rate of spread from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_FBP_FROS</code> flank rate of spread from FBP calculations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_ROS</code> rate of spread calculated from Dr. Richards' ellipse equations (metres per minute)
			<li><code>CWFGM_FIRE_STAT_CFB</code> crown fraction burned (unitless), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_CFC</code> crown fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_SFC</code> surface fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_TFC</code> total fuel consumption (kg/m2), based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_FI</code> fire intensity, based on ROS from Dr. Richards' ellipse equations
			<li><code>CWFGM_FIRE_STAT_FLAMELENGTH</code> flame length (metres), based on ROS from Dr. Richards' ellipse equations
			</ul>
		\param greater_equal Lower range of values
		\param less_than Upper range of values
		\param stats Calculated statistic value
		\sa ICWFGM_Scenario::GetStatsPercentage

		\retval E_POINTER The address provided for time or stats is invalid
		\retval S_OK Successful
		\retval ERROR_NO_DATA|ERROR_SEVERITY_WARNING Nothing has been initialized yet
		\retval ERROR_FIRE_INVALID_TIME If the time is invalid
		\retval ERROR_FIRE_SCENARIO_UNKNOWN This fire object is not attached to the scenario specified by the scenario parameter
		\retval SUCCESS_FIRE_NOT_STARTED Fire not  yet started (stats is set to 0)
		\retval ERROR_FIRE_STAT_UNKNOWN If stat does not resolve to a known statistic
		\retval ERROR_SCENARIO_BAD_STATE If the function is run without a running scenario
		\retval E_OUTOFMEMORY Insufficient memory
	*/
	virtual NO_THROW HRESULT GetStatsPercentage(const std::uint32_t fire, HSS_Time::WTime* time, const std::uint16_t stat, const double greater_equal, const double less_than, double *stats) const;

	/** This method returns a fuel type that the scenario will use at the specified location and time.  Scenarios may modify or replace fuels based on attribute grids assigned to it, and this function considers those rules.
		\param pt The x and y coordinate for the point
		\param time Specified GMT time since January 1, 1600
		\param fuel Returned fuel type.  May return NULL.
		\param fuel_valid Returned fuel validity. 
		\param cache_bbox: May be NULL. If provided, then the rectangle is filled or adjusted to indicate the maximum size of the cell containing the requested point which is constant and uniform in content and value.
		\sa ICWFGM_Scenario::GetFuelData

		\retval E_POINTER The address provided for fuel is invalid
		\retval S_OK Successful
		\retval ERROR_NO_DATA|ERROR_SEVERITY_WARNING Nothing has been initialized yet
		\retval ERROR_FIRE_INVALID_TIME If the time is invalid
		\retval ERROR_SCENARIO_BAD_STATE If the function is run without a running scenario
		\retval E_OUTOFMEMORY Insufficient memory
	*/
	virtual NO_THROW HRESULT GetFuelData(const XY_Point& pt,const HSS_Time::WTime& time,boost::intrusive_ptr<ICWFGM_Fuel>* fuel, bool* fuel_valid, CCWFGM_FuelOverrides *overrides, XY_Rectangle* cache_bbox);
	/** Exports fires generated from a specific ignition at a specific simulated time, given options. 
		\param set Identifies a specific ignition. If NULL, then all fires at the time specified are exported
		\param start_time Start time for which to export at, specified GMT time since Midnight January 1, 1600.  Only displayable time steps will be output.
		\param end_time End time for which to export at, specified GMT time since Midnight January 1, 1600.  Only displayable time steps will be output.
		\param flags
			<ul>
			<li><code>SCENARIO_EXPORT_SUBSET_EXTERIOR</code> If true, then only exterior perimeters are exported.  If false, then all exterior and interior perimeters are exported.
			<li><code>SCENARIO_EXPORT_SUBSET_ACTIVE</code> If true, then active portions of the perimeters are exported as polylines.  If false, then all portions of the perimeters are exported (as polygons).
			<li><code>SCENARIO_EXPORT_COMBINE_SET</code> If true, then multiple fire perimeters will be merged together.  If false, then boundaries among the fires will persist.
			</ul>
		\param driver_name Identifies file format.  Refer to GDAL documentation for supported formats
		\param projection Projection file name
		\param file_path Vector data file name
		\param rules Array of SExportRule rules defining specific details of what to include in the exported file.
		\sa ICWFGM_Scenario::ExportFires

		\retval E_POINTER The address provided for time, driver_name or file_path is invalid
		\retval S_OK Successful
		\retval ERROR_NO_DATA|ERROR_SEVERITY_WARNING Nothing has been initialized yet
		\retval ERROR_FIRE_INVALID_TIME If the time is invalid
		\retval SUCCESS_FIRE_NOT_STARTED Fire not y et started (stats is set to 0)
		\retval ERROR_SCENARIO_BAD_STATE If the function is run without a running scenario
		\retval E_INVALIDARG Invalid parameter
		\retval ERROR_GRID_UNINITIALIZED Impossible to export
		\retval S_FALSE Unspecified error
		\retval E_FAIL Unspecidifed error
	*/
	virtual NO_THROW HRESULT ExportFires(const CCWFGM_Ignition *set, HSS_Time::WTime &start_time, HSS_Time::WTime &end_time, std::uint16_t flags,const std::string &driver_name, const std::string &projection, const std::string &file_path, const class ScenarioExportRules *rules) const;
	virtual NO_THROW HRESULT ExportCriticalPath(const ICWFGM_Asset* asset, const std::uint32_t index, const std::uint16_t flags, const std::string& driver_name, const std::string& projection, const std::string& file_path, const ScenarioExportRules* rules) const;
	virtual NO_THROW HRESULT BuildCriticalPath(const ICWFGM_Asset* asset, const std::uint32_t index, const std::uint16_t flags, MinListTempl<CriticalPath>& polyset, const ScenarioExportRules* rules) const;

	virtual NO_THROW HRESULT GetPercentileClassCount(unsigned char *count);
	virtual NO_THROW HRESULT GetPercentileFuel(unsigned char indexFuel, _GUID *defaultFuel);
	virtual NO_THROW HRESULT SetPercentileValue(const _GUID *defaultFuel, unsigned char fireDescription, double s);
	virtual NO_THROW HRESULT GetPercentileValue(const _GUID *defaultFuel, unsigned char fireDescription, double *s);
	virtual NO_THROW HRESULT RSI(const _GUID *clsId, double RSIin, double CFBin, double *RSIout) override;
#ifndef DOXYGEN_IGNORE_CODE
public:
	virtual std::int32_t serialVersionUid(const SerializeProtoOptions& options) const noexcept override;
	virtual WISE::FireEngineProto::CwfgmScenario* serialize(const SerializeProtoOptions& options) override;
	virtual CCWFGM_Scenario *deserialize(const google::protobuf::Message& proto, std::shared_ptr<validation::validation_object> valid, const std::string& name) override;
	virtual std::optional<bool> isdirty(void) const noexcept override { return m_bRequiresSave; }

public:
	MinListTempl<VectorEngineNode>								m_vectorEngineList;
	boost::intrusive_ptr<ICWFGM_Target>							m_windTarget, m_vectorTarget;
	unsigned long												m_windTargetIndex, m_windTargetSubIndex, m_vectorTargetIndex, m_vectorTargetSubIndex;

protected:
	struct Impl;
	std::experimental::propagate_const<std::unique_ptr<Impl>>	m_impl;

public:
	boost::intrusive_ptr<ICWFGM_GridEngine>						m_gridEngine;
	Layer														*m_layerThread;

	ICWFGM_FWI													*m_fwi;
	WTimeManager												*m_timeManager;

protected:
	std::string													m_loadWarning;

	double				m_perimeterResolution;	// maximum distance between 2 vertices on a fire perimeter
	double				m_spatialThreshold;		// maximum distance a vertex can travel in one time step

public:
	WTime				m_startTime,
						m_endTime,
						m_ignitionOverrideTime;

	WTimeSpan			m_displayInterval,			// how often the output is to be stopped and re-displayed
						m_temporalThreshold_Accel;	// same as m_temporalThreshold but during ignition acceleration (for points)

	WTimeSpan			m_dt;
	double				m_dx, m_dy, m_dwd, m_owd, m_ovd, m_dvd;
	double				m_growthPercentile;
	double				m_perimeterSpacing;		// minimum distance between 2 vertices on a fire perimeter

	double				perimeterResolution(const double area) const;
	double				spatialThreshold(const double area) const;
	double				m_minimumROS;			// minimum ROS (in m/min, default is 1mm/min) that can support fire growth - less than this and fire will be stopped and flagged with FP_FLAG_NO_ROS

	double				m_specifiedFMC;
	double				m_defaultElevation;
	double				m_ignitionSize;

	ScenarioPercentile	m_sp;
	StopCondition		m_sc;

	std::uint64_t		m_optionFlags;
	std::uint32_t		m_globalAssetOperation;

	std::uint16_t		m_initialVertexCount;

	std::uint32_t		m_threadingNumProcessors;	// NOT saved in the FGM as it should be a machine-dependent setting
	CRWThreadSemaphore	m_lock;				// This grants access to this CWFGM_Scenario object.
											// If it's write-only then that means an option or parameter is being changed, or the objects associated
											// with the scenario are being locked or unlocked, or the m_scenario object is being created/destroyed.
											// It, however, does NOT mean that it controls shared/exclusive access to the m_scenario object - if it
											// exists and is not in a state of change (being destroyed, etc.) then they are all shared accesses.  It
											// is then up to the m_scenario object to control shared/exclusive access to itself.

private:
	bool m_bRequiresSave;

#endif
private:
	HRESULT lockObjects();
	HRESULT checkWxGridObjects();
	HRESULT checkWxGridObject(const WTimeSpan &duration, std::uint32_t option, std::vector<uint16_t> *s);
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif
