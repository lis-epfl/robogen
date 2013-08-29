/*
 * PassiveHingeRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/PassiveHingeRepresentation.h"

namespace robogen {

PassiveHingeRepresentation::PassiveHingeRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation, 1) {
}

PassiveHingeRepresentation::~PassiveHingeRepresentation() {
}

std::vector<std::string> PassiveHingeRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> PassiveHingeRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
