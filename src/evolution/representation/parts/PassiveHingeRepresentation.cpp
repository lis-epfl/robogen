/*
 * PassiveHingeRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/representation/parts/PassiveHingeRepresentation.h"

namespace robogen {

PassiveHingeRepresentation::PassiveHingeRepresentation(std::string id,
		int orientation) : PartRepresentation(id, orientation, 1,
				"passivehinge") {
}

PassiveHingeRepresentation::~PassiveHingeRepresentation() {
}

boost::shared_ptr<PartRepresentation>
PassiveHingeRepresentation::cloneSubtree(){
	boost::shared_ptr<PartRepresentation> theClone(
			new PassiveHingeRepresentation(this->getId(),
					this->getOrientation()));
	// deep copy all children
	for (int i=1; i<=this->getArity(); i++){
		if (this->getChild(i))
			theClone->setChild(i,this->getChild(i)->cloneSubtree());
	}
	return theClone;
}

std::vector<std::string> PassiveHingeRepresentation::getMotors(){
	return std::vector<std::string>(0);
}

std::vector<std::string> PassiveHingeRepresentation::getSensors(){
	return std::vector<std::string>(0);
}

} /* namespace robogen */
