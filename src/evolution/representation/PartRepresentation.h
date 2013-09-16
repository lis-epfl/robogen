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
#include <map>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include "robogen.pb.h"

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
	PartRepresentation(std::string id, int orientation, int arity,
			const std::string type);

	/**
	 * copy constructor replacement with deep copy of children
	 * copy constructor is not very useful, as we mostly want to copy derived
	 * classes from pointers to the base class. Thus, we'll use this trick:
	 * http://stackoverflow.com/questions/5731217/how-to-copy-create-derived-
	 * class-instance-from-a-pointer-to-a-polymorphic-base-c
	 * @return new derived instance of the part
	 * @todo recursive pattern robust for bigger robots?
	 */
	virtual boost::shared_ptr<PartRepresentation> cloneSubtree() = 0;

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
	 * @return type of the part
	 */
	std::string &getType();

	/**
	 * @return arity = number of child slots of part
	 */
	int getArity();

	/**
	 * @return amount of children, recursively: Total amount of dependent parts
	 */
	int numDescendants();

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

	/**
	 * Add subtree to given body message.
	 * @param bodyMessage message of the body to be completed with the subtree
	 * @param amIRoot if set to true, will dsignate itself as root part
	 * @todo recursive pattern robust for bigger robots?
	 */
	void addSubtreeToBodyMessage(robogenMessage::Body *bodyMessage,
			bool amIRoot);

	/**
	 * @param parent parent to be set
	 */
	void setParent(PartRepresentation *parent);

	/**
	 * @return parent part
	 */
	PartRepresentation *getParent();

	/**
	 * Set slot id on parent part
	 */
	void setPosition(int position);

	/**
	 * @return slot occupied on parent part
	 */
	int getPosition();

protected:
	/**
	 * As of now, only derived classes need to call this for cloning, so it's
	 * protected
	 */
	int getOrientation();

	/**
	 * Map of all parameters, necessary for serialization. Protected, so that
	 * derived classes can insert parameters. Initialization in constructor
	 * would have been cleaner, but, with C++, alas, impossible.
	 */
	std::map<std::string, double> params_;

private:
	/**
	 * Identifier string (name) of this part
	 */
	std::string id_;

	/**
	 * Type, as required for protobuf serialization of derived classes
	 */
	std::string type_;

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

	/**
	 * Parent body part - raw pointer as present (or NULL) by design
	 */
	PartRepresentation *parent_;

	/**
	 * Slot occupied on parent part
	 */
	int position_;

};

} /* namespace robogen */
#endif /* PARTREPRESENTATION_H_ */
