/*
 * @(#) TcpSocket.cpp   1.0   Feb 25, 2013
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
#include <boost/bind.hpp>
#include "utils/network/TcpSocket.h"

namespace robogen {

TcpSocket::TcpSocket() {

}

TcpSocket::~TcpSocket() {

}

bool TcpSocket::create(int port) {

   this->port_ = port;

   try {
      this->acceptor_.reset(
            new boost::asio::ip::tcp::acceptor(this->ioService_,
                  boost::asio::ip::tcp::endpoint(
                        boost::asio::ip::address::from_string("0.0.0.0"), port),
                  true));
   } catch (std::exception& e) {
      this->exceptionHandler(e);
      return false;
   }

   return true;
}

bool TcpSocket::accept() {

   try {
      this->socket_.reset(new boost::asio::ip::tcp::socket(this->ioService_));
      this->acceptor_->accept(*this->socket_);
   } catch (std::exception& e) {
      this->exceptionHandler(e);
      return false;
   }
   return true;

}

bool TcpSocket::open(const std::string& ip, int port) {

   try {
      this->socket_.reset(new boost::asio::ip::tcp::socket(this->ioService_));
      this->socket_->connect(
            boost::asio::ip::tcp::endpoint(
                  boost::asio::ip::address::from_string(ip), port));
   } catch (std::exception& e) {
      this->exceptionHandler(e);
      return false;
   }
   return true;

}

bool TcpSocket::read(std::vector<unsigned char>& buffer, size_t bytesToRead) {

   buffer.resize(bytesToRead);
   size_t bytesRead = boost::asio::read(*this->socket_,
         boost::asio::buffer(buffer));
   if (bytesRead != bytesToRead) {
      return false;
   }
   return true;
}

bool TcpSocket::close() {
   try {

      if (this->acceptor_ != NULL) {
         this->acceptor_->close();
      }

      if (this->socket_ != NULL) {
         this->socket_->close();
      }
      this->ioService_.stop();
   } catch (std::exception& e) {
      this->exceptionHandler(e);
      return false;
   }
   return true;
}

bool TcpSocket::write(std::vector<unsigned char>& buffer) {
   size_t bytesSent = boost::asio::write(*this->socket_,
         boost::asio::buffer(buffer));
   return (bytesSent == buffer.size());
}

void TcpSocket::exceptionHandler(std::exception& e) {
   std::cerr << "Exception: " << e.what() << "\n";
}

void TcpSocket::interrupt() {
   this->ioService_.stop();
}

}
