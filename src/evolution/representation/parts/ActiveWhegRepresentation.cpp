/*
 * ActiveWhegRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/representation/parts/ActiveWhegRepresentation.h"

namespace robogen {

ActiveWhegRepresentation::ActiveWhegRepresentation(std::string id,
		int orientation, double radius) : PartRepresentation(id, orientation,0,
				"activewheg"){
	params_["radius"] = radius;
}

ActiveWhegRepresentation::~ActiveWhegRepresentation() {
}

boost::shared_ptr<PartRepresentation> ActiveWhegRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new ActiveWhegRepresentation(this->getId(),
					this->getOrientation(), params_["radius"]));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> ActiveWhegRepresentation::getMotors(){
	std::vector<std::string> motors;
	motors.push_back(this->getId());
	return motors;
}

std::vector<std::string> ActiveWhegRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
