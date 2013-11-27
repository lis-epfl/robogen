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
#include <boost/weak_ptr.hpp>
#include "robogen.pb.h"

namespace robogen {

/**
 * Part representation to be used for evolution. More lightweight than the
 * part representation of the simulator, and implements evolution-specific
 * methods.
 */
class PartRepresentation {

public:

	static std::map<class PartRepresentation, std::string> PART_REPRESENTATION_TYPE_MAP;

	/**
	 * Intializes a body part.
	 * The orientation takes a value in [0,3] that correspond to a rotation {0, 90, 180, 270} degrees.
	 *
	 * @param id name of the part
	 * @param orientation orientation of the part when attached to parent part
	 * @param arity arity of the part
	 * @param type of the part
	 * @param params parameters of the part
	 */
	PartRepresentation(std::string id, unsigned int orientation, unsigned int arity,
			const std::string& type, const std::vector<double>& params,
			const std::vector<std::string>& motors,
			const std::vector<std::string>& sensors);

	/**
	 * Clone subtree
	 * copy constructor is not very useful, as we mostly want to copy derived
	 * classes from pointers to the base class. Thus, we'll use this trick:
	 * http://stackoverflow.com/questions/5731217/how-to-copy-create-derived-
	 * class-instance-from-a-pointer-to-a-polymorphic-base-c
	 * @return new derived instance of the part
	 * @todo recursive pattern robust for bigger robots?
	 */
	boost::shared_ptr<PartRepresentation> cloneSubtree();

	/**
	 * Destructor
	 */
	virtual ~PartRepresentation();

	/**
	 * @return identifier of part
	 */
	void setId(std::string newid);

	/**
	 * @return identifier of part
	 */
	std::string &getId();

	/**
	 * @return type of the part
	 */
	std::string &getType();

	/**
	 * @return the parameters of the part
	 */
	std::vector<double> getParams();

	/**
	 * @return arity = number of child slots of part
	 */
	unsigned int getArity();

	/**
	 * @return amount of children, recursively: Total amount of dependent parts
	 */
	unsigned int numDescendants();

	/**
	 * @param n slot of the child part to get
	 */
	boost::shared_ptr<PartRepresentation> getChild(unsigned int n);

	/**
	 * @param n slot of the child part to be set/replaced
	 * @param part shared pointer to a part which is to be included at the slot
	 * @return true if successful
	 */
	bool setChild(unsigned int n, boost::shared_ptr<PartRepresentation> part);

	/**
	 * Factory pattern to create derived classes, i.e. body part representations
	 */
	static boost::shared_ptr<PartRepresentation> create(char type,
			std::string id, unsigned int orientation,
			std::vector<double> params);

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
	void setParent(PartRepresentation* parent);

	/**
	 * @return parent part
	 */
	PartRepresentation* getParent();

	/**
	 * Set slot id on parent part
	 */
	void setPosition(int position);

	/**
	 * @return slot occupied on parent part
	 */
	int getPosition();

	/**
	 * @return the number of children
	 */
	unsigned int getChildrenCount();

	/**
	 * @return the positions of the free slots
	 */
	std::vector<unsigned int> getFreeSlots();

	/**
	 * @return the ids of all ancestors of this part
	 */
	std::vector<std::string> getAncestorsIds();

	/**
	 * @return the ids of all descendants of this part
	 */
	std::vector<std::string> getDescendantsIds();

	/**
	 * @return vector containing motor identifiers
	 */
	std::vector<std::string> getMotors();

	/**
	 * @return vector containing sensor identifiers
	 */
	std::vector<std::string> getSensors();

	/**
	 * @return the orientation of this part
	 */
	unsigned int getOrientation();

	/**
	 * @param orientation of part
	 */
	void setOrientation(unsigned int orientation);

	/**
	 * Print recursively the body tree representation
	 */
	void toString(std::stringstream& str, unsigned int depth);

private:

	/**
	 * Identifier string (name) of this part
	 */
	std::string id_;

	/**
	 * orientation of the part when attached to parent part
	 */
	unsigned int orientation_;

	/**
	 * Arity: Amount of available children slots
	 */
	unsigned int arity_;

	/**
	 * Type, as required for protobuf serialization of derived classes
	 */
	std::string type_;

	/**
	 * Children of this part in the body tree
	 */
	std::vector<boost::shared_ptr<PartRepresentation> > children_;

	/**
	 * Parent body part - raw pointer as present (or NULL) by design
	 */
	PartRepresentation* parent_;

	/**
	 * Slot occupied on parent part
	 */
	int position_;

	/**
	 * Parameters of the body part
	 */
	std::vector<double> params_;

	/**
	 * Motors
	 */
	std::vector<std::string> motors_;

	/**
	 * Sensors
	 */
	std::vector<std::string> sensors_;

};

} /* namespace robogen */
#endif /* PARTREPRESENTATION_H_ */
