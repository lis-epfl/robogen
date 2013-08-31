/*
 * PassiveCardanRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/parts/PassiveCardanRepresentation.h"

namespace robogen {

PassiveCardanRepresentation::PassiveCardanRepresentation(std::string id,
		int orientation) : PartRepresentation(id,orientation,1,"passivecardan"){
}

PassiveCardanRepresentation::~PassiveCardanRepresentation() {
}

boost::shared_ptr<PartRepresentation>
PassiveCardanRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new PassiveCardanRepresentation(this->getId(),
					this->getOrientation()));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> PassiveCardanRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> PassiveCardanRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
