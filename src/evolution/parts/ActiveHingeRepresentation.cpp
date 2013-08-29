/*
 * ActiveHingeRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/ActiveHingeRepresentation.h"

namespace robogen {

ActiveHingeRepresentation::ActiveHingeRepresentation(std::string id,
		int orientation) : PartRepresentation(id,orientation,1) {
}

ActiveHingeRepresentation::~ActiveHingeRepresentation() {
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
