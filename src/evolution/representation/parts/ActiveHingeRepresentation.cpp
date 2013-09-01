/*
 * ActiveHingeRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/representation/parts/ActiveHingeRepresentation.h"

namespace robogen {

ActiveHingeRepresentation::ActiveHingeRepresentation(std::string id,
		int orientation) : PartRepresentation(id,orientation,1,"activehinge") {
}

ActiveHingeRepresentation::~ActiveHingeRepresentation() {
}

boost::shared_ptr<PartRepresentation> ActiveHingeRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new ActiveHingeRepresentation(this->getId(),
					this->getOrientation()));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> ActiveHingeRepresentation::getMotors(){
	std::vector<std::string> motors;
	motors.push_back(this->getId());
	return motors;
}

std::vector<std::string> ActiveHingeRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
