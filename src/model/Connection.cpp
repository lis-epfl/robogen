/*
 * @(#) Connection.cpp   1.0   Aug 26, 2013
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
#include "model/Connection.h"
#include <map>
#include <sstream>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <model/Model.h>

namespace robogen{

ConnectionException::ConnectionException(const std::string &w) :
		std::runtime_error(w){}

Connection::Connection(robogenMessage::BodyConnection &c,
		std::map<std::string, boost::shared_ptr<Model> > &map) :
		fromSlot_(c.srcslot()), toSlot_(c.destslot()){

	// identify from node
	std::map<std::string, boost::shared_ptr<Model> >::iterator it =
			map.find(c.src());
	if (it == map.end()){
		std::stringstream ss;
		ss << "Error initializing Connection: Source id" << c.src() << " could"\
				" not be resolved in body parts map passed to constructor.";
		throw ConnectionException(ss.str());
	}
	from_ = (it->second);

	// identify to node
	it = map.find(c.dest());
	if (it == map.end()){
		std::stringstream ss;
		ss << "Error initializing Connection: Destination id" << c.dest() <<
				" could not be resolved in body parts map passed to "\
				"constructor.";
		throw ConnectionException(ss.str());
	}
	if (it->second == from_){
		std::stringstream ss;
		ss << "Error initializing Connection: Destination id" << c.dest() <<
				" is same as source id!";
		throw ConnectionException(ss.str());
	}
	to_ = (it->second);

	// TODO verify proper slot id's: Model::arity() though this is kind of done
	// internally in the Model derived classes' getSlot() already

}

void Connection::toOde(){

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
