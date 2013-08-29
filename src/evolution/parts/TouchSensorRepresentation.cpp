/*
 * TouchSensorRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/TouchSensorRepresentation.h"

namespace robogen {

TouchSensorRepresentation::TouchSensorRepresentation(std:: string id,
		int orientation) : PartRepresentation(id,orientation,0) {
}

TouchSensorRepresentation::~TouchSensorRepresentation() {
}

std::vector<std::string> TouchSensorRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> TouchSensorRepresentation::getSensors(){
	std::vector<std::string> sensors;
	sensors.push_back(this->getId() + "-left");
	sensors.push_back(this->getId() + "-right");
	return sensors;
}

} /* namespace robogen */
