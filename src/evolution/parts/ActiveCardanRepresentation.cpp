/*
 * ActiveCardanRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/ActiveCardanRepresentation.h"

namespace robogen {

ActiveCardanRepresentation::ActiveCardanRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation, 1) {
}

ActiveCardanRepresentation::~ActiveCardanRepresentation() {
}

boost::shared_ptr<PartRepresentation> ActiveCardanRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new ActiveCardanRepresentation(this->getId(),
					this->getOrientation()));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> ActiveCardanRepresentation::getMotors(){
	std::vector<std::string> motors;
	motors.push_back(this->getId() + "-tilt-1");
	motors.push_back(this->getId() + "-tilt-2");
	return motors;
}

std::vector<std::string> ActiveCardanRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
