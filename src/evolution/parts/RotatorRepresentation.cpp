/*
 * RotatorRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/RotatorRepresentation.h"

namespace robogen {

RotatorRepresentation::RotatorRepresentation(std::string id, int orientation) :
	PartRepresentation(id, orientation, 1){
}

RotatorRepresentation::~RotatorRepresentation() {
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
