/*
 * @(#) PartRepresentation.h   1.0   Aug 28, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
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

#ifndef PARTREPRESENTATION_H_
#define PARTREPRESENTATION_H_

#include <string>
#include <vector>
#include <stdexcept>
#include <boost/shared_ptr.hpp>

namespace robogen {

/**
 * Exception class for things that go wrong with part representation.
 */
class PartRepresentationException : public std::runtime_error {
public:
	PartRepresentationException(const std::string& w);
};

/**
 * Part representation to be used for evolution. More lightweight than the
 * part representation of the simulator, and implements evolution-specific
 * methods.
 */
class PartRepresentation {
public:
	/**
	 * @param id name of the part
	 * @param orientation orientation of the part when attached to parent part
	 */
	PartRepresentation(std::string id, int orientation, int arity);

	// TODO copy constructor

	virtual ~PartRepresentation();

	/**
	 * @return vector containing motor identifiers
	 */
	virtual std::vector<std::string> getMotors() = 0;

	/**
	 * @return vector containing sensor identifiers
	 */
	virtual std::vector<std::string> getSensors() = 0;

	/**
	 * @return identifier of part
	 */
	std::string &getId();

	/**
	 * @param n slot of the child part to get
	 */
	boost::shared_ptr<PartRepresentation> getChild(int n);

	/**
	 * @param n slot of the child part to be set/replaced
	 * @param part shared pointer to a part which is to be included at the slot
	 * @return shared pointer to part previously present at slot
	 */
	boost::shared_ptr<PartRepresentation> setChild(int n,
			boost::shared_ptr<PartRepresentation> part);

	/**
	 * Factory pattern to create derived classes, i.e. body part representations
	 */
	static boost::shared_ptr<PartRepresentation> create(char type, std::string
			id, int orientation, std::vector<double> params);

private:
	/**
	 * Identifier string (name) of this part
	 */
	std::string id_;

	/**
	 * Arity: Amount of available children slots
	 */
	const int arity_;

	/**
	 * orientation of the part when attached to parent part
	 */
	int orientation_;

	/**
	 * Children of this part in the body tree
	 */
	std::vector<boost::shared_ptr<PartRepresentation> > children_;
};

} /* namespace robogen */
#endif /* PARTREPRESENTATION_H_ */
