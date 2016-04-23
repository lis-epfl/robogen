/*
 * @(#) Socket.h   1.0   Aug 11 2015
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Guillaume Leclerc (guillaume.leclerc@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */


#include <iostream>
#include <string>
#include <vector>
#


#ifndef SOCKET_H_
#define SOCKET_H_

namespace robogen {

class Socket {
public:
	Socket();
	virtual ~Socket();
public :
	/**
	    * Create a tcp socket, awaiting connections on the specified port
	    * @param port the port number
	    * @return true if the operation completed succesfull, false otherwise
	    */
	   virtual bool create(int port) = 0;

	   /**
	    * Wait until a client connected to the socket.
	    * Blocking call.
	    */
	   virtual bool accept() = 0;

	   /**
	    * Connects to the specified socket
	    * @param ip the ip address
	    * @param port the port number
	    * @return true if the operation completed succesfull, false otherwise
	    */
	   virtual bool open(const std::string& ip, int port) = 0;

	   /**
	    * Read exactly the specified amount of data from the socket.
	    * @param buffer
	    * @param bytesToRead
	    */
	   virtual bool read(std::vector<unsigned char>& buffer, size_t bytesToRead) = 0;

	   /**
	    * Write the buffer on the unix socket
	    * @param buffer
	    */
	   virtual bool write(std::vector<unsigned char>& buffer) = 0;

	   /**
	    * Closes the socket
	    * @return true if the operation completed succesfull, false otherwise
	    */
	   virtual bool close() = 0;

	   /**
	    * Interrupt the socket, terminating any blocking call
	    */
	   virtual void interrupt() = 0;
};

} /* namespace robogen */

#endif /* SOCKET_H_ */
