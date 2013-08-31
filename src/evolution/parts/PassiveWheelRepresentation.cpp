/*
 * PassiveWheelRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/PassiveWheelRepresentation.h"

namespace robogen {

PassiveWheelRepresentation::PassiveWheelRepresentation(std::string id,
		int orientation, double radius) :
		PartRepresentation(id,orientation,0, "passivewheel"){
	params_["radius"] = radius;
}

PassiveWheelRepresentation::~PassiveWheelRepresentation() {
}

boost::shared_ptr<PartRepresentation>
PassiveWheelRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new PassiveWheelRepresentation(this->getId(),
					this->getOrientation(), params_["radius"]));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> PassiveWheelRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> PassiveWheelRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
