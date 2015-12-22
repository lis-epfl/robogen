/*
 * @(#) FakeSocket.h   1.0 Aug 11, 2015
 *
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
#ifndef FAKEJSSOCKET_H_
#define FAKEJSSOCKET_H_

#include <utils/network/Socket.h>

namespace robogen {

class FakeJSSocket: public Socket {
public:
	FakeJSSocket();
	virtual ~FakeJSSocket();
	virtual bool create(int port);
	virtual bool accept();
	virtual bool open(const std::string& ip, int port);
	virtual bool read(std::vector<unsigned char>& buffer, size_t bytesToRead);
	virtual bool write(std::vector<unsigned char>& buffer);
	virtual bool close();
	virtual void interrupt();
	std::vector<unsigned char> getContent();
private :
	std::vector<unsigned char> innerBuffer;
};

} /* namespace robogen */

#endif /* FAKEJSSOCKET_H_ */
