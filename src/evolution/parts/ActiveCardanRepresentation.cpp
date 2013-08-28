/*
 * ActiveCardanRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/ActiveCardanRepresentation.h"

namespace robogen {

ActiveCardanRepresentation::ActiveCardanRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation) {
}

ActiveCardanRepresentation::~ActiveCardanRepresentation() {
}

int ActiveCardanRepresentation::arity(){
	return 1;
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
