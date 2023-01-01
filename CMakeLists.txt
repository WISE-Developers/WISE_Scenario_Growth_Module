cmake_minimum_required (VERSION 3.13)
if (MSVC)
set (CMAKE_SYSTEM_VERSION 10.0.22000.0)
endif (MSVC)
project (FireEngine VERSION ${PROMETHEUS_VERSION} DESCRIPTION "Fire Engine Library")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)

SET(LOWLEVEL_INCLUDE_DIR "error" CACHE STRING "The path to the includes from HSS LowLevel")
SET(WTIME_INCLUDE_DIR "error" CACHE STRING "The path to the WTime libraries includes")
SET(MULTITHREAD_INCLUDE_DIR "error" CACHE STRING "The path to the includes from HSS Multithread")
SET(MATH_INCLUDE_DIR "error" CACHE STRING "The path to HSS_Math include files")
SET(GEOGRAPHY_INCLUDE_DIR "error" CACHE STRING "The path to HSS_Math include files")
SET(THIRD_PARTY_INCLUDE_DIR "error" CACHE STRING "The path to third party include files")
SET(ERROR_CALC_INCLUDE_DIR "error" CACHE STRING "The path to the error calc include files")
SET(FWI_INCLUDE_DIR "error" CACHE STRING "The path to the FWI include files")
SET(WEATHER_INCLUDE_DIR "error" CACHE STRING "The path to the weather include files")
SET(FUEL_INCLUDE_DIR "error" CACHE STRING "The path to the fuel include files")
SET(GRID_INCLUDE_DIR "error" CACHE STRING "The path to the grid include files")
SET(REDAPP_INCLUDE_DIR "error" CACHE STRING "The path to the REDappWrapper include files")
SET(GDAL_INCLUDE_DIR "error" CACHE STRING "The path to the GDAL include files")
SET(GSL_INCLUDE_DIR "error" CACHE STRING "The path to the GSL include files")
SET(PROTOBUF_INCLUDE_DIR "error" CACHE STRING "The path to the protobuf include files")

find_library(FOUND_MULTITHREAD_LIBRARY_PATH NAMES Multithread REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_LOWLEVEL_LIBRARY_PATH NAMES LowLevel REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_WTIME_LIBRARY_PATH NAMES WTime REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_MATH_LIBRARY_PATH NAMES hssmath REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_GEOGRAPHY_LIBRARY_PATH NAMES hssgeography REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_PROTOBUF_LIBRARY_PATH NAMES protobuf libprotobuf REQUIRED PATHS ${PROTOBUF_LIBRARY_DIR})
find_library(FOUND_ERROR_CALC_LIBRARY_PATH NAMES ErrorCalc REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_FWI_LIBRARY_PATH NAMES fwi REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_FUEL_LIBRARY_PATH NAMES fuel REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_GRID_LIBRARY_PATH NAMES grid REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_REDAPP_LIBRARY_PATH NAMES REDappWrapper REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_WEATHER_LIBRARY_PATH NAMES weather REQUIRED PATHS ${LOCAL_LIBRARY_DIR})
find_library(FOUND_GDAL_LIBRARY_PATH NAMES gdal gdal_i REQUIRED PATHS ${GDAL_LIBRARY_DIR})

if (MSVC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX -DPROTOBUF_USE_DLLS -DBOOST_ALL_NO_LIB -DBOOST_ALL_DYN_LINK -D_CRT_SECURE_NO_WARNINGS /Zc:__cplusplus")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD -D_AFXDLL")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd -D_AFXDLL /W4")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS atomic chrono system iostreams REQUIRED PATHS ${BOOST_LIBRARY_DIR})
else ()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

find_package(Boost COMPONENTS chrono system iostreams REQUIRED PATHS ${BOOST_LIBRARY_DIR})
endif (MSVC)

message(WARNING ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_NO_MFC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -DDEBUG")

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

add_library(fireengine SHARED
    cpp/cwfgmFire.pb.cc
    cpp/cwfgmScenario.pb.cc
    cpp/CWFGM_Fire.cpp
    cpp/CWFGM_Fire.Serialize.cpp
    cpp/CWFGM_Scenario.cpp
    cpp/CWFGM_Scenario.Serialize.cpp
    cpp/excel_tinv.cpp
    cpp/firefront.cpp
    cpp/firepoint.cpp
    cpp/firestatecache.cpp
    cpp/FireStateGrow.cpp
    cpp/FireStateSmooth.cpp
    cpp/firestatestats.cpp
    cpp/FireStateTrack.cpp
    cpp/Percentile.cpp
    cpp/scenario.cpp
    cpp/scenario.delaunay.cpp
    cpp/scenario.stats.cpp
    cpp/ScenarioAsset.cpp
    cpp/ScenarioExportRules.cpp
    cpp/ScenarioIgnition.cpp
    cpp/ScenarioTimeStep.cpp
    cpp/StopCondition.cpp
)

target_include_directories(fireengine
    PUBLIC ${MATH_INCLUDE_DIR}
    PUBLIC ${FWI_INCLUDE_DIR}
    PUBLIC ${LOWLEVEL_INCLUDE_DIR}
    PUBLIC ${MULTITHREAD_INCLUDE_DIR}
    PUBLIC ${WTIME_INCLUDE_DIR}
    PUBLIC ${PROTOBUF_INCLUDE_DIR}
    PUBLIC ${FUEL_INCLUDE_DIR}
    PUBLIC ${GEOGRAPHY_INCLUDE_DIR}
    PUBLIC ${ERROR_CALC_INCLUDE_DIR}
    PUBLIC ${WEATHER_INCLUDE_DIR}
    PUBLIC ${GDAL_INCLUDE_DIR}
    PUBLIC ${GRID_INCLUDE_DIR}
    PUBLIC ${REDAPP_INCLUDE_DIR}
    PUBLIC ${THIRD_PARTY_INCLUDE_DIR}
    PUBLIC ${Boost_INCLUDE_DIRS}
    PUBLIC ${GSL_INCLUDE_DIR}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(fireengine PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})
set_target_properties(fireengine PROPERTIES SOVERSION ${CMAKE_PROJECT_VERSION_MAJOR})
set_target_properties(fireengine PROPERTIES DEFINE_SYMBOL "FIRE_EXPORTS")

set_target_properties(fireengine PROPERTIES
    PUBLIC_HEADER include/CWFGM_Fire.h
    PUBLIC_HEADER include/CWFGM_Scenario.h
    PUBLIC_HEADER include/cwfgmFire.pb.h
    PUBLIC_HEADER include/cwfgmScenario.pb.h
    PUBLIC_HEADER include/excel_tinv.h
    PUBLIC_HEADER include/FireEngine_ext.h
    PUBLIC_HEADER include/FireEngine.h
    PUBLIC_HEADER include/firefront.h
    PUBLIC_HEADER include/firepoint.h
    PUBLIC_HEADER include/firestatecache.h
    PUBLIC_HEADER include/firestatestats.h
    PUBLIC_HEADER include/Precentile.h
    PUBLIC_HEADER include/scenario.h
    PUBLIC_HEADER include/ScenarioAsset.h
    PUBLIC_HEADER include/ScenarioExportRules.h
    PUBLIC_HEADER include/ScenarioIgnition.h
    PUBLIC_HEADER include/ScenarioTimeStep.h
    PUBLIC_HEADER include/SExportRule.h
    PUBLIC_HEADER include/StopCondition.h
)

target_link_libraries(fireengine ${FOUND_GDAL_LIBRARY_PATH} ${Boost_LIBRARIES} ${FOUND_PROTOBUF_LIBRARY_PATH})
target_link_libraries(fireengine ${FOUND_FWI_LIBRARY_PATH} ${FOUND_FUEL_LIBRARY_PATH} ${FOUND_GRID_LIBRARY_PATH} ${FOUND_REDAPP_LIBRARY_PATH} ${FOUND_WEATHER_LIBRARY_PATH})
target_link_libraries(fireengine ${FOUND_WTIME_LIBRARY_PATH} ${FOUND_LOWLEVEL_LIBRARY_PATH} ${FOUND_MULTITHREAD_LIBRARY_PATH} ${FOUND_ERROR_CALC_LIBRARY_PATH} ${FOUND_MATH_LIBRARY_PATH} ${FOUND_GEOGRAPHY_LIBRARY_PATH})
if (MSVC)
else ()
target_link_libraries(fireengine -lstdc++fs)
endif (MSVC)
