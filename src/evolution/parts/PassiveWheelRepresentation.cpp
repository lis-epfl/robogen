/*
 * PassiveWheelRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/PassiveWheelRepresentation.h"

namespace robogen {

PassiveWheelRepresentation::PassiveWheelRepresentation(std::string id,
		int orientation) : PartRepresentation(id,orientation) {
}

PassiveWheelRepresentation::~PassiveWheelRepresentation() {
}

int PassiveWheelRepresentation::arity(){
	return 0;
}

std::vector<std::string> PassiveWheelRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> PassiveWheelRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
