/*
 * ParametricBarJointRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/ParametricBarJointRepresentation.h"

namespace robogen {

ParametricBarJointRepresentation::ParametricBarJointRepresentation(
		std::string id, int orientation, double length, double inclination,
		double rotation) : PartRepresentation(id, orientation, 1,
				"parametricbrick"){
	params_["length"] = length;
	params_["inclinationangle"] = inclination;
	params_["rotationangle"] = rotation;
}

ParametricBarJointRepresentation::~ParametricBarJointRepresentation() {
}

boost::shared_ptr<PartRepresentation>
ParametricBarJointRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new ParametricBarJointRepresentation(this->getId(),
					this->getOrientation(), params_["length"],
					params_["inclinationangle"], params_["rotationangle"]));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> ParametricBarJointRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> ParametricBarJointRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
