/*
 * @(#) Connection.cpp   1.0   Aug 26, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013
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
#include "model/Connection.h"
#include <map>
#include <sstream>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <model/Model.h>

namespace robogen{

Connection::Connection(){
}

Connection::Connection(boost::shared_ptr<Model> from, int fromSlot,
			boost::shared_ptr<Model> to, int toSlot):from_(from),
					fromSlot_(fromSlot), to_(to), toSlot_(toSlot) {}

bool Connection::init(const robogenMessage::BodyConnection &c,
		std::map<std::string, unsigned int> &map,
		std::vector<boost::shared_ptr<Model> > &vec){

	fromSlot_ = c.srcslot();
	toSlot_ = c.destslot();

	// identify from node
	std::map<std::string, unsigned int>::iterator it =
			map.find(c.src());
	if (it == map.end()){
		std::cout << "Error initializing Connection: Source id" << c.src() <<
				" could not be resolved in body parts map passed to init." <<
				std::endl;
		return false;
	}
	from_ = vec[(it->second)]; // TODO catch out of bounds

	// identify to node
	it = map.find(c.dest());
	if (it == map.end()){
		std::cout << "Error initializing Connection: Destination id" << c.dest()
				<< " could not be resolved in body parts map passed to "\
				"constructor." << std::endl;
		return false;
	}
	if (vec[it->second] == from_){
		std::cout << "Error initializing Connection: Destination id" << c.dest() <<
				" is same as source id!" << std::endl;
		return false;
	}
	to_ = vec[(it->second)];
	return true;
}

boost::shared_ptr<Model> Connection::getFrom(){
	return from_;
}

int Connection::getFromSlot(){
	return fromSlot_;
}

boost::shared_ptr<Model> Connection::getTo(){
	return to_;
}

int Connection::getToSlot(){
	return toSlot_;
}

} /* namespace robogen */
