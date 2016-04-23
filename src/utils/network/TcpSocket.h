/*
 * @(#) TcpSocket.h   1.0   Feb 25, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#ifndef ROBOGEN_TCP_SOCKET_H_
#define ROBOGEN_TCP_SOCKET_H_

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <utils/network/Socket.h>

namespace robogen {

/**
 * Wrapper for a tcp socket
 */
class TcpSocket : public Socket{

public:

   /**
    * Constructor
    */
   TcpSocket();

   /**
    * Destructor
    */
   virtual ~TcpSocket();

   /**
    * Create a tcp socket, awaiting connections on the specified port
    * @param port the port number
    * @return true if the operation completed succesfull, false otherwise
    */
   virtual bool create(int port);

   /**
    * Wait until a client connected to the socket.
    * Blocking call.
    */
   virtual bool accept();

   /**
    * Connects to the specified socket
    * @param ip the ip address
    * @param port the port number
    * @return true if the operation completed succesfull, false otherwise
    */
   virtual bool open(const std::string& ip, int port);

   /**
    * Read exactly the specified amount of data from the socket.
    * @param buffer
    * @param bytesToRead
    */
   virtual bool read(std::vector<unsigned char>& buffer, size_t bytesToRead);

   /**
    * Write the buffer on the unix socket
    * @param buffer
    */
   virtual bool write(std::vector<unsigned char>& buffer);

   /**
    * Closes the socket
    * @return true if the operation completed succesfull, false otherwise
    */
   virtual bool close();

   /**
    * Interrupt the socket, terminating any blocking call
    */
   virtual void interrupt();

private:

   /**
    * Called every time an exception has been detected
    */
   void exceptionHandler(std::exception& e);

   /**
    * Boost IO Service (handles OS calls)
    */
   boost::asio::io_service ioService_;

   /**
    * Acceptor (server-side of the socket)
    */
   boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
   /**
    * Unix socket
    */
   boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;

   /**
    * The ip address of the socket
    */
   std::string ip_;

   /**
    * The port
    */
   int port_;
};

}

#endif /* ROBOGEN_TCP_SOCKET_H_ */
