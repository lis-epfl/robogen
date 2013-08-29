/*
 * FixedBrickRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/FixedBrickRepresentation.h"

namespace robogen {

FixedBrickRepresentation::FixedBrickRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation,5) {
}

FixedBrickRepresentation::~FixedBrickRepresentation() {
}

std::vector<std::string> FixedBrickRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> FixedBrickRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
