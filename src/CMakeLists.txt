# CMake Required Version
cmake_minimum_required (VERSION 2.8)

# Project Name
project (RobogenSimulator)

# Add customized CMAKE modules
set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_SOURCE_DIR}/cmake")

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
	add_definitions(-ansi -pedantic -Wno-long-long -Wall -Wextra -Wformat=2
	                -Wredundant-decls -Wwrite-strings -Wmissing-include-dirs 
	                -Wswitch-enum -Wuninitialized 
	                -Wswitch-default -Winit-self -Wshadow -Wfloat-equal -fPIC)
	
	# Debug Flags 
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -DDEBUG")
	
	# Release flags
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops -finline-functions -fomit-frame-pointer -DNDEBUG")
	

elseif(WIN32)

	# Debug Flags 
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -DDEBUG /MP")
	
	# Release flags
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG /MP")

endif()


if (CMAKE_BUILD_TYPE STREQUAL "Release")
	include(OptimizeForArchitecture)
	OptimizeForArchitecture("auto")
endif ()


# Get the list of all the source code
file (GLOB_RECURSE
     ROBOGEN_SIMULATOR_SRC
     brain/*.c
     config/*.cpp
     model/*.cpp
     render/*.cpp
     scenario/*.cpp
     utils/*.cpp
     Robot.cpp
     Robogen.cpp
)
 

find_package(Boost COMPONENTS date_time graph system program_options regex)
find_package(Protobuf REQUIRED)
find_package(ZLIB REQUIRED)
find_package(PNG REQUIRED)
find_package(OpenSceneGraph REQUIRED osg osgViewer osgDB osgGA osgTerrain)

include_directories(${CMAKE_SOURCE_DIR} ${ODE_INCLUDE_PATH} ${OPENSCENEGRAPH_INCLUDE_DIRS} ${PNG_INCLUDE_PATH} ${ZLIB_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} ${PNG_INCLUDE_DIRS})

file (GLOB_RECURSE
     ROBOGEN_SIMULATOR_PROTO
     *.proto
     )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ROBOGEN_SIMULATOR_PROTO})

set(ROBOGEN_DEPENDENCIES ${ODE_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${ZLIB_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${PNG_LIBRARIES} pthread)

message(STATUS ${ROBOGEN_DEPENDENCIES})

add_library(robogen ${ROBOGEN_SIMULATOR_SRC} ${PROTO_SRCS} )
target_link_libraries(robogen ${ROBOGEN_DEPENDENCIES})

add_executable(robogen-test test.cpp)
target_link_libraries(robogen-test robogen ${ROBOGEN_DEPENDENCIES})

add_executable(robogen-simulator server.cpp)
target_link_libraries(robogen-simulator robogen ${ROBOGEN_DEPENDENCIES})

add_executable(robogen-file-viewer viewer/FileViewer.cpp)
target_link_libraries(robogen-file-viewer robogen ${ROBOGEN_DEPENDENCIES})

add_executable(robogen-server-viewer viewer/ServerViewer.cpp)
target_link_libraries(robogen-server-viewer robogen ${ROBOGEN_DEPENDENCIES})

add_executable(robogen-server-viewer-test viewer/ServerViewerTest.cpp)
target_link_libraries(robogen-server-viewer-test robogen ${ROBOGEN_DEPENDENCIES})
