/*
 * FixedBrickRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/FixedBrickRepresentation.h"

namespace robogen {

FixedBrickRepresentation::FixedBrickRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation,5) {
}

FixedBrickRepresentation::~FixedBrickRepresentation() {
}

boost::shared_ptr<PartRepresentation> FixedBrickRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new FixedBrickRepresentation(this->getId(),
					this->getOrientation()));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> FixedBrickRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> FixedBrickRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
