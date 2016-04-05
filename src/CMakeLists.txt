# CMake Required Version
cmake_minimum_required (VERSION 2.8)

# Project Name
project (RoboGen)

option(TARGET_JS "Build the project in javascript" OFF)
option(ENABLE_QT "Enable QT for scriptable scenarios" ON)
option(MAKE_JS_TEST "Make JavaScript test" OFF)

message(STATUS "${EM_ODE_INCLUDE_DIR}")

set(EMSCRIPTEN_PATH CACHE FILEPATH "The path for the emscripten tool suite")
if ("${EMSCRIPTEN_PATH}" STREQUAL "")
	set(EMSCRIPTEN_PATH $ENV{EMSCRIPTEN})
endif()

# Add customized CMAKE modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

if (WIN32)

file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/../resources")
file(COPY "${CMAKE_SOURCE_DIR}/brain/NeuralNetwork.h" DESTINATION "${CMAKE_SOURCE_DIR}/../resources")

else()

execute_process(
    COMMAND ${CMAKE_SOURCE_DIR}/../build_utils/wrap_file.sh ${CMAKE_SOURCE_DIR}/brain/NeuralNetwork.h ${CMAKE_SOURCE_DIR}/brain/NeuralNetwork.template
) 
endif()

# Pass source dir to preprocessor
add_definitions(-DSOURCE_DIR=${CMAKE_SOURCE_DIR})

# Tell NEAT to use boost random
add_definitions(-DUSE_BOOST_RANDOM)

if(TARGET_JS) 
	# macro to find subdirectories
	MACRO(SUBDIRLIST result curdir)
		FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
		SET(dirlist "")
		FOREACH(child ${children})
			IF(IS_DIRECTORY ${curdir}/${child})
				LIST(APPEND dirlist "${curdir}/${child}")
			ENDIF()
		ENDFOREACH()
		SET(${result} ${dirlist})
	ENDMACRO()

	#list containing all the llvm-archives required to link the project
	set (ROBOGEN_JS_REQUIRED_ARCHIVES "jansson" "ode" "osg" "boost_timer" "boost_system" "boost_chrono" "boost_regex" "boost_serialization" "boost_date_time" "boost_graph" "boost_program_options" "boost_wserialization" "boost_filesystem" "protobuf")

	#list required include path to compile the source files
	set (ROBOGEN_JS_REQUIRED_INCLUDES "ode" "jansson" "osg" "boost" "protobuf")

	message(STATUS "Targeting the browser (javascript)")

	#Checking for emscripten 
	if("${EMSCRIPTEN_PATH}" STREQUAL "" OR NOT EXISTS "${EMSCRIPTEN_PATH}") 
		message(FATAL_ERROR "Emscripten compiler is not available, make sure your environement variables are set")
	else()
		message(STATUS "Emscripten found at : ${EMSCRIPTEN_PATH}")
	endif()

	#Checking for osg
	set (ROBOGEN_JS_DEP_BUILD_DIR "../../build" CACHE FILEPATH "The directory where we can find headers and llvm-BC for libraries")
	get_filename_component(ROBOGEN_JS_DEP_BUILD_DIR "${ROBOGEN_JS_DEP_BUILD_DIR}" ABSOLUTE)
	if (NOT EXISTS "${ROBOGEN_JS_DEP_BUILD_DIR}")
		message (FATAL_ERROR "The directory ${ROBOGEN_JS_DEP_BUILD_DIR} does not exists")
	else()
		message (STATUS "The library build directory is : ${ROBOGEN_JS_DEP_BUILD_DIR}")
	endif()

	message (STATUS "Searching for llvm library archives")
	set (ROBOGEN_JS_LIBRARIES "")
	file (GLOB_RECURSE ROBOGEN_JS_LIBRARIES_GLOB "${ROBOGEN_JS_DEP_BUILD_DIR}/lib/*.a")
	foreach (LIBRARY_TO_FIND ${ROBOGEN_JS_REQUIRED_ARCHIVES})
		set (FOUND OFF)
		foreach (CURRENT_LIB ${ROBOGEN_JS_LIBRARIES_GLOB})
			string (FIND "${CURRENT_LIB}" "${LIBRARY_TO_FIND}.a" _INDEX)
			if (_INDEX GREATER -1)
				message (STATUS "${LIBRARY_TO_FIND} found at : ${CURRENT_LIB}")
				list (APPEND ROBOGEN_JS_LIBRARIES "${CURRENT_LIB}")
				set (FOUND ON)
			endif()
		endforeach()
		if (NOT ${FOUND}) 
			message(FATAL_ERROR "${LIBRARY_TO_FIND} was not found")
		endif()
	endforeach()
	message(STATUS "All required libraries found")

	message(STATUS "Finding include paths")
	set (ROBOGEN_JS_INCLUDES "")
	SUBDIRLIST(AVAILABLE_INCLUDES_DIR "${ROBOGEN_JS_DEP_BUILD_DIR}/include")

	foreach (INCLUDE_TO_FIND ${ROBOGEN_JS_REQUIRED_INCLUDES})
		set (FOUND OFF)
		foreach (CURRENT_INCLUDE ${AVAILABLE_INCLUDES_DIR})
			string (FIND "${CURRENT_INCLUDE}" "${INCLUDE_TO_FIND}" _INDEX)
			if (_INDEX GREATER -1)
				message (STATUS "Include path for ${INCLUDE_TO_FIND} found here : ${CURRENT_INCLUDE}")
				list (APPEND ROBOGEN_JS_INCLUDES "${CURRENT_INCLUDE}")
				set (FOUND ON)
			endif()
		endforeach()
		if (NOT ${FOUND})
			message(FATAL_ERROR "${INCLUDE_TO_FIND} was not found")
		endif()
	endforeach()
	message (STATUS "All include path found")

	set(RELEASE_OPT_FLAG "-Oz")

	include_directories (${CMAKE_SOURCE_DIR} ${ROBOGEN_JS_INCLUDES})
	add_definitions(-DTARGET_JS -DEMSCRIPTEN -DDISABLE_HEIGHT_MAP "-Wno-warn-absolute-paths" --bind "-s DISABLE_EXCEPTION_CATCHING=0 ")
	set(CMAKE_EXE_LINKER_FLAGS "--bind -s NO_EXIT_RUNTIME=1 ${EXPORTED_FLAG}")
	set(CMAKE_CXX_FLAGS_RELEASE "-Oz" CACHE STRING "Robogen OPT") 
	set(CMAKE_CXX_FLAGS_DEBUG "-Wall" CACHE STRING "Robogen Wall") 
	if (CMAKE_BUILD_TYPE STREQUAL "Release")
          set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${RELEASE_OPT_FLAG} -s DISABLE_EXCEPTION_CATCHING=0 -s ALLOW_MEMORY_GROWTH=1")
		message("${CMAKE_EXE_LINKER_FLAGS}")
		add_definitions("${RELEASE_OPT_FLAG}")
	else ()
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s DISABLE_EXCEPTION_CATCHING=0")
	endif()

	file(GLOB_RECURSE ROBOGEN_JS_SOURCES 
		arduino/*.cpp
		brain/*.c
		evolution/engine/*.cpp
		evolution/engine/neat/*.cpp
		evolution/neat/*.cpp
		evolution/representation/*.cpp
		evolution/representation/parts/*.cpp
		config/*.cpp
		model/*.cpp
		printing/*.cpp
		utils/*.cpp
		Robot.cpp
		Robogen.cpp
		PartList.cpp
		Simulator.cpp
		robogen.pb.cc

		# I added a compilation flag to disable height map and then OsgDB is not needed anymore
		scenario/*.cpp 

		viewer/FileViewerLog.cpp
		viewer/WebGLLogger.cpp
		viewer/JSViewer.cpp

		##########################################
		#     FAILING :
		##########################################
		#
		# render/*.cpp Requires
		# => Requires OsgViewer
		#
		# viewer/*.cpp
		# => Requires OsgGA
		# 
		##########################################
	)


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()
	

add_executable(app "robogen_js.cpp" ${ROBOGEN_JS_SOURCES})
	target_link_libraries(app ${ROBOGEN_JS_LIBRARIES})

else()
	message(STATUS "Targeting the local architecture")

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
		scripting/*.cpp
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
	find_package(OpenSceneGraph REQUIRED osg osgViewer osgDB osgGA osgUtil osgTerrain)
	find_package(Jansson REQUIRED)
	if (ENABLE_QT)
		if (QT_PATH)
			set (CMAKE_PREFIX_PATH "${QT_PATH}")
		endif()
		find_package(Qt5Core)
		if (Qt5Core_FOUND)
			set(CMAKE_AUTOMOC ON)
			add_definitions("-DQT5_ENABLED")
			message(STATUS "Qt enabled and found! Scripted scenarios will be available!")
		else()
			message(STATUS "Qt5Core Not Found, will not be able to use scripted scenarios!")
		endif()
	else()
		message(STATUS "Qt disabled, will not be able to use scripted scenarios!")
	endif()

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
	add_library(robogen ${ROBOGEN_SRC} ${PROTO_SRCS} )
	target_link_libraries(robogen ${ROBOGEN_DEPENDENCIES})

	# Evolver executable
	add_executable(robogen-evolver Evolver.cpp)
	target_link_libraries(robogen-evolver robogen ${ROBOGEN_DEPENDENCIES})

	# Robogen simulator server
	add_executable(robogen-server RobogenServer.cpp)
	target_link_libraries(robogen-server robogen ${ROBOGEN_DEPENDENCIES})

	# Robot Viewer (from file)
	add_executable(robogen-file-viewer viewer/FileViewer.cpp)
	target_link_libraries(robogen-file-viewer robogen ${ROBOGEN_DEPENDENCIES})

	if (Qt5Core_FOUND)
		if(MAKE_JS_TEST)
			message(STATUS "MAKING js-test")
			add_executable(js-test JsTest.cpp Exposable.h)
			target_link_libraries(js-test robogen ${ROBOGEN_DEPENDENCIES})
			qt5_use_modules(js-test Core Script)
		endif()
		
		qt5_use_modules(robogen Core Script)
	endif()

	# Tests
	#add_executable(robogen-server-viewer-test viewer/ServerViewerTest.cpp)
	#target_link_libraries(robogen-server-viewer-test robogen ${ROBOGEN_DEPENDENCIES})

	#add_executable(robogen-test test.cpp)
	#target_link_libraries(robogen-test robogen ${ROBOGEN_DEPENDENCIES})

endif()
