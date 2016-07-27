/*
 * @(#) SocketIOConnectionListener.h   1.0   Jun 6, 2016
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
#ifndef ROBOGEN_SOCKETIOCONNECTIONLISTENER_H_
#define ROBOGEN_SOCKETIOCONNECTIONLISTENER_H_

#include <mutex>
#include <condition_variable>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include "Robogen.h"
#include "sio_client.h"

namespace robogen {




class SocketIOConnectionListener {

public:

	SocketIOConnectionListener(std::mutex *mutex);

	~SocketIOConnectionListener();

	sio::socket::ptr getSocket();

	void waitForConnect();

	void waitForDisconnect();

	void connect(const std::string uri);

private:

	void onConnected();

	void onClose(sio::client::close_reason const& reason);

	void onFail();

	void onReconnecting();

	boost::shared_ptr<sio::client> handler_;
	bool connected_ = false;
	std::mutex *lock_;
	std::condition_variable_any cond_;

};

}

#endif /* ROBOGEN_SOCKETIOCONNECTIONLISTENER_H_ */
