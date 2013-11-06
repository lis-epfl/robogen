/*
 * @(#) Connection.h   1.0   Aug 26, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Titus Cieslewski
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

#ifndef CONNECTION_H_
#define CONNECTION_H_

#include <map>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include "model/Model.h"
#include "robogen.pb.h"

namespace robogen {

class ConnectionException : public std::runtime_error{
public:
	ConnectionException(const std::string &w);
};

/**
 * \brief Representation of connection between two parts
 * This class serves to explicitly represent a connection between two parts
 * of the robot. Thus, the robot representation in the simulator is independent
 * of whatever compact representation has been used before.
 */
class Connection {
public:
	/**
	 * Error-less contructor
	 */
	Connection();

	Connection(boost::shared_ptr<Model> from, int fromSlot,
			boost::shared_ptr<Model> to, int toSlot);

	/**
	 * Creates a connection from the specified connection message
	 * @param c Connection message to be used (uses string id's)
	 * @param map mapping from string id to index in vector of body part
	 * @param vec std::vector containing the body part shared pointers
	 */
	bool init(const robogenMessage::BodyConnection &c,
			std::map<std::string, unsigned int> &map,
			std::vector<boost::shared_ptr<Model> > &vec);

	/**
	 * TODO remove once toOde is in place
	 * @return from body part
	 */
	boost::shared_ptr<Model> getFrom();

	/**
	 * TODO remove once toOde is in place
	 * @return from slot id
	 */
	int getFromSlot();

	/**
	 * TODO remove once toOde is in place
	 * @return to body part
	 */
	boost::shared_ptr<Model> getTo();

	/**
	 * TODO remove once toOde is in place
	 * @return to slot id
	 */
	int getToSlot();

private:
	/**
	 * Parent body part
	 */
	boost::shared_ptr<Model> from_;

	/**
	 * Slot used for connection at parent body part
	 */
	unsigned int fromSlot_;

	/**
	 * Child body part
	 */
	boost::shared_ptr<Model> to_;

	/**
	 * Slot used for connection at child body part
	 */
	unsigned int toSlot_;
};

} /* namespace robogen */
#endif /* CONNECTION_H_ */
