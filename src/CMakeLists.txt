# CMake Required Version
cmake_minimum_required (VERSION 2.8)

# Project Name
project (RoboGen)

# Add customized CMAKE modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Pass source dir to preprocessor
add_definitions(-DSOURCE_DIR=${CMAKE_SOURCE_DIR})

# Include Open Dynamics Engine (ODE)
if ("${ODE_INCLUDE_PATH}" STREQUAL "" OR "${ODE_LIBRARIES}" STREQUAL "")

	set(ODE_INCLUDE_PATH $ENV{ODE_INCLUDE_PATH})
	set(ODE_LIBRARIES $ENV{ODE_LIBRARIES})
	
	if ("${ODE_INCLUDE_PATH}" STREQUAL "" OR "${ODE_LIBRARIES}" STREQUAL "")
	
		# Try to load them from configuration file
		if(EXISTS "${CMAKE_SOURCE_DIR}/cmake/CustomPath.cmake")
			include(CustomPath)
			message(STATUS "Included CustomPath.cmake: ${ODE_INCLUDE_PATH} - ${ODE_LIBRARIES}")
		endif()
			
		if ("${ODE_INCLUDE_PATH}" STREQUAL "" OR "${ODE_LIBRARIES}" STREQUAL "")
			message(FATAL_ERROR "The 'ODE_INCLUDE_PATH' or 'ODE_LIBRARIES' environment variable were not set and the configuration file 'CustomPath.cmake' is not present")
		endif()
		
	endif()
	
endif()


if(UNIX)
	# Compiler options
	if (CMAKE_BUILD_TYPE STREQUAL "Release")
		add_definitions(-std=c++11 -pedantic -fPIC )
	else()
		# Turn on warnings in debug mode
		add_definitions(-std=c++11 -pedantic -Wno-long-long -Wall -Wextra -Wformat=2
		                -Wredundant-decls -Wwrite-strings -Wmissing-include-dirs 
		                -Wswitch-enum -Wuninitialized 
		                -Wswitch-default -Winit-self -Wshadow -Wfloat-equal -fPIC )
	endif()
	# Debug Flags 
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -DDEBUG")
	
	# Release flags
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops -finline-functions -fomit-frame-pointer -DNDEBUG")
	

elseif(WIN32)

	# Debug Flags 
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -DDEBUG /MP")
	
	# Release flags
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG /MP")
	
	SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /FORCE:MULTIPLE" )

endif()


if (CMAKE_BUILD_TYPE STREQUAL "Release")
	include(OptimizeForArchitecture)
	OptimizeForArchitecture("auto")
endif ()


# Get the list of all the source code
file (GLOB_RECURSE
     ROBOGEN_SRC
     arduino/*.cpp
     brain/*.c
     config/*.cpp
     evolution/*.cpp
     model/*.cpp
     printing/*.cpp
     render/*.cpp
     scenario/*.cpp
     utils/*.cpp
     viewer/*.cpp
     Robot.cpp
     Robogen.cpp
     PartList.cpp
     Simulator.cpp
)


find_package(Boost COMPONENTS chrono date_time graph system program_options regex filesystem timer thread serialization)
find_package(Protobuf REQUIRED)
find_package(ZLIB REQUIRED)
find_package(PNG REQUIRED)
find_package(OpenSceneGraph REQUIRED osg osgViewer osgDB osgGA osgTerrain osgUtil)
find_package(Jansson REQUIRED)
find_package(Qt5Core REQUIRED)


include_directories(${CMAKE_SOURCE_DIR} ${ODE_INCLUDE_PATH} ${OPENSCENEGRAPH_INCLUDE_DIRS} ${PNG_INCLUDE_PATH} ${ZLIB_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} ${PNG_INCLUDE_DIRS} ${JANSSON_INCLUDE_DIRS} )

file (GLOB_RECURSE
     ROBOGEN_PROTO
     *.proto
     )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ROBOGEN_PROTO})

set(ROBOGEN_DEPENDENCIES ${ODE_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${ZLIB_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${PNG_LIBRARIES} ${JANSSON_LIBRARIES})


message(STATUS ${ROBOGEN_DEPENDENCIES})

# Robogen base library
#add_library(robogen ${ROBOGEN_SRC} ${PROTO_SRCS} )
#target_link_libraries(robogen ${ROBOGEN_DEPENDENCIES})

# Evolver executable
#add_executable(robogen-evolver Evolver.cpp)
#target_link_libraries(robogen-evolver robogen ${ROBOGEN_DEPENDENCIES})

# Robogen simulator server
#add_executable(robogen-server RobogenServer.cpp)
#target_link_libraries(robogen-server robogen ${ROBOGEN_DEPENDENCIES})

# Robot Viewer (from file)
#add_executable(robogen-file-viewer viewer/FileViewer.cpp)
#target_link_libraries(robogen-file-viewer robogen ${ROBOGEN_DEPENDENCIES})

set(CMAKE_AUTOMOC ON)

add_executable(js-test JsTest.cpp Exposable.h)
target_link_libraries(js-test ${ROBOGEN_DEPENDENCIES})
qt5_use_modules(js-test Core Script)


# Tests
#add_executable(robogen-server-viewer-test viewer/ServerViewerTest.cpp)
#target_link_libraries(robogen-server-viewer-test robogen ${ROBOGEN_DEPENDENCIES})

#add_executable(robogen-test test.cpp)
#target_link_libraries(robogen-test robogen ${ROBOGEN_DEPENDENCIES})

