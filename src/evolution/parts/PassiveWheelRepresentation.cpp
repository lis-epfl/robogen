/*
 * PassiveWheelRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/PassiveWheelRepresentation.h"

namespace robogen {

PassiveWheelRepresentation::PassiveWheelRepresentation(std::string id,
		int orientation, double radius) : PartRepresentation(id,orientation,0),
		radius_(radius){
}

PassiveWheelRepresentation::~PassiveWheelRepresentation() {
}

std::vector<std::string> PassiveWheelRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> PassiveWheelRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
