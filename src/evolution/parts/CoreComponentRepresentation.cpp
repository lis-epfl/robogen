/*
 * CoreComponentRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/CoreComponentRepresentation.h"

namespace robogen {

CoreComponentRepresentation::CoreComponentRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation, 5,
				"corecomponent") {
}

CoreComponentRepresentation::~CoreComponentRepresentation() {
}

boost::shared_ptr<PartRepresentation> CoreComponentRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new CoreComponentRepresentation(this->getId(),
					this->getOrientation()));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
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
