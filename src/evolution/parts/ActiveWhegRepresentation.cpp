/*
 * ActiveWhegRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/ActiveWhegRepresentation.h"

namespace robogen {

ActiveWhegRepresentation::ActiveWhegRepresentation(std::string id,
		int orientation, double radius) : PartRepresentation(id, orientation,0),
		radius_(radius){
}

ActiveWhegRepresentation::~ActiveWhegRepresentation() {
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
