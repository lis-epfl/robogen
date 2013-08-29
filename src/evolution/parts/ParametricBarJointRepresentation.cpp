/*
 * ParametricBarJointRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/ParametricBarJointRepresentation.h"

namespace robogen {

ParametricBarJointRepresentation::ParametricBarJointRepresentation(
		std::string id, int orientation, double length, double inclination,
		double rotation) : PartRepresentation(id, orientation, 1),
		length_(length), inclination_(inclination), rotation_(rotation){
}

ParametricBarJointRepresentation::~ParametricBarJointRepresentation() {
}

std::vector<std::string> ParametricBarJointRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> ParametricBarJointRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
