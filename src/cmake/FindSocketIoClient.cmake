# - Try to find the socket.io client
# Once done this will define
#
#  SOCKET_IO_CLIENT_FOUND - system has the socket.io c++ client
#  SOCKET_IO_CLIENT_INCLUDE_DIRS - the socket.io c++ client include directory
#  SOCKET_IO_CLIENT_LIBRARIES - Link these to use the socket.io c++ client
#
# The ROBOGEN Framework
# Copyright © 2012-2016 Joshua Auerbach
#
# Laboratory of Intelligent Systems, EPFL
#
# This file is part of the ROBOGEN Framework.
#
# The ROBOGEN Framework is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License (GPL)
# as published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#

if (SOCKET_IO_CLIENT_INCLUDE_DIRS AND SOCKET_IO_CLIENT_LIBRARIES)
  # in cache already
  set(SOCKET_IO_CLIENT_FOUND TRUE)
else ()

	find_path(SOCKET_IO_CLIENT_INCLUDE_DIRS
		NAMES
			sio_client.h
			sio_message.h
			sio_socket.h
		PATHS
			/usr/include
			/usr/local/include
			/opt/local/include
			${CMAKE_SOURCE_DIR}/../socket.io-client-cpp/build/include
	)

	find_library(SOCKET_IO_CLIENT_LIBRARIES
	    NAMES
		sioclient
		sioclient_tls
	    PATHS
		/usr/lib
		/usr/local/lib
		/opt/local/lib
		${CMAKE_SOURCE_DIR}/../socket.io-client-cpp/build/lib/Release
  	)


	include(FindPackageHandleStandardArgs)
	find_package_handle_standard_args(SOCKET_IO_CLIENT DEFAULT_MSG SOCKET_IO_CLIENT_LIBRARIES SOCKET_IO_CLIENT_INCLUDE_DIRS)

	# show the vars only in advanced view
	mark_as_advanced(SOCKET_IO_CLIENT_LIBRARIES SOCKET_IO_CLIENT_INCLUDE_DIRS)

endif ()


