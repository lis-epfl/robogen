/*
 * PassiveCardanRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/PassiveCardanRepresentation.h"

namespace robogen {

PassiveCardanRepresentation::PassiveCardanRepresentation(std::string id,
		int orientation) : PartRepresentation(id,orientation,1) {
}

PassiveCardanRepresentation::~PassiveCardanRepresentation() {
}

std::vector<std::string> PassiveCardanRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> PassiveCardanRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
