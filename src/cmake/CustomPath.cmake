# This file is generally not needed and is included only if libraries are installed in non-standard locations
# Uncomment and set appropriately the following paths

# Set the path to your utils folder
#set(UTILS_PATH "C:/Users/lis/Documents/Robogen/Simulator/utils")

####################################################################
# Do not edit under this line
####################################################################

if (WIN32)

	# ZLIB
	set (ZLIB_ROOT ${UTILS_PATH})

	# PNG
	set (PNG_PNG_INCLUDE_DIR ${UTILS_PATH})
	set (PNG_LIBRARY "${UTILS_PATH}/lib/libpng.lib")

	# OSG
	set(ENV{OSG_DIR} ${UTILS_PATH} )

	# Protobuf
	set (PROTOBUF_INCLUDE_DIR "${UTILS_PATH}/include")
	set (PROTOBUF_LIBRARY "${UTILS_PATH}/lib/libprotobuf.lib")
	set (PROTOBUF_LIBRARY_DEBUG "${UTILS_PATH}/lib/libprotobufd.lib")
	set (PROTOBUF_PROTOC_EXECUTABLE "${UTILS_PATH}/bin/protoc.exe")

	# BOOST
	set (BOOST_ROOT ${UTILS_PATH})
	set (Boost_USE_STATIC_LIBS ON)

	# ODE
	set (ODE_INCLUDE_PATH "${UTILS_PATH}/include")
	set (ODE_LIBRARIES "${UTILS_PATH}/lib/ode_double.lib")

endif()
