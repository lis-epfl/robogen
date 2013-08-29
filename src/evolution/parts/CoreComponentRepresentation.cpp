/*
 * CoreComponentRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/CoreComponentRepresentation.h"

namespace robogen {

CoreComponentRepresentation::CoreComponentRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation, 5) {
}

CoreComponentRepresentation::~CoreComponentRepresentation() {
}

std::vector<std::string> CoreComponentRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> CoreComponentRepresentation::getSensors(){
	std::vector<std::string> sensors;
	sensors.push_back("x-acceleration");
	sensors.push_back("y-acceleration");
	sensors.push_back("z-acceleration");
	sensors.push_back("Pitch");
	sensors.push_back("Roll");
	sensors.push_back("Yaw");
	return sensors;
}

} /* namespace robogen */
