/*
 * ActiveWheelRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/ActiveWheelRepresentation.h"

namespace robogen {

ActiveWheelRepresentation::ActiveWheelRepresentation(std::string id,
		int orientation, double radius) : PartRepresentation(id,orientation,0),
		radius_(radius){
}

ActiveWheelRepresentation::~ActiveWheelRepresentation() {
}

std::vector<std::string> ActiveWheelRepresentation::getMotors(){
	std::vector<std::string> motors;
	motors.push_back(this->getId());
	return motors;
}

std::vector<std::string> ActiveWheelRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
