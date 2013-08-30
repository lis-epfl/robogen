/*
 * LightSensorRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/LightSensorRepresentation.h"

namespace robogen {

LightSensorRepresentation::LightSensorRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation, 0) {
}

LightSensorRepresentation::~LightSensorRepresentation() {
}

boost::shared_ptr<PartRepresentation> LightSensorRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new LightSensorRepresentation(this->getId(),
					this->getOrientation()));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> LightSensorRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> LightSensorRepresentation::getSensors(){
	std::vector<std::string> sensors;
	sensors.push_back(this->getId());
	return sensors;
}

} /* namespace robogen */
