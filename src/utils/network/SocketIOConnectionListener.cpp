/*
 * @(#) SocketIOConnectionListener.cpp   1.0   Jun 6, 2016
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach
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

#include "utils/network/SocketIOConnectionListener.h"

namespace robogen {

SocketIOConnectionListener::SocketIOConnectionListener(std::mutex *mutex) :
	connected_(false), lock_(mutex) {
	handler_.reset(new sio::client());
	handler_->set_open_listener(
			std::bind(&SocketIOConnectionListener::onConnected, this));
	handler_->set_close_listener(
			std::bind(&SocketIOConnectionListener::onClose, this,
					std::placeholders::_1));
	handler_->set_fail_listener(
			std::bind(&SocketIOConnectionListener::onFail, this));
	handler_->set_reconnecting_listener(
			std::bind(&SocketIOConnectionListener::onReconnecting, this));
}

SocketIOConnectionListener::~SocketIOConnectionListener() {
	handler_->sync_close();
	handler_->clear_con_listeners();
}

sio::socket::ptr SocketIOConnectionListener::getSocket() {
	return handler_->socket();
}

void SocketIOConnectionListener::waitForConnect() {
	lock_->lock();
	while(!connected_) {
		cond_.wait(*lock_);
	}
	lock_->unlock();
}

void SocketIOConnectionListener::waitForDisconnect() {
	lock_->lock();
	while(connected_) {
		cond_.wait(*lock_);
	}
	lock_->unlock();
}


void SocketIOConnectionListener::connect(const std::string uri) {
	handler_->connect(uri);
}

void SocketIOConnectionListener::onConnected() {
	std::cout << "********************* sio connected "
			<< "*********************" << std::endl;
	lock_->lock();
	connected_ = true;
	cond_.notify_all();
	lock_->unlock();
}

void SocketIOConnectionListener::onClose(sio::client::close_reason const&
		reason) {
	std::cout << "********************* sio closed "
			<< "*********************" << std::endl;
	exitRobogen(EXIT_SUCCESS);
}

void SocketIOConnectionListener::onFail() {

	std::cerr << "********************* sio failed "
			<< "*********************" << std::endl;
	exitRobogen(EXIT_FAILURE);
}

void SocketIOConnectionListener::onReconnecting() {

	std::cout << "********************* sio reconnecting "
			<< "*********************" << std::endl;
	lock_->lock();
	connected_ = false;
	cond_.notify_all();
	lock_->unlock();
}

}
