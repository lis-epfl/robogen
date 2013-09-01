/*
 * RotatorRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/representation/parts/RotatorRepresentation.h"

namespace robogen {

RotatorRepresentation::RotatorRepresentation(std::string id, int orientation) :
	PartRepresentation(id, orientation, 1, "activerotator"){
}

RotatorRepresentation::~RotatorRepresentation() {
}

boost::shared_ptr<PartRepresentation> RotatorRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new RotatorRepresentation(this->getId(),
					this->getOrientation()));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> RotatorRepresentation::getMotors(){
	std::vector<std::string> motors;
	motors.push_back(this->getId());
	return motors;
}

std::vector<std::string> RotatorRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
