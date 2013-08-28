/*
 * PassiveCardanRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef PASSIVECARDANREPRESENTATION_H_
#define PASSIVECARDANREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class PassiveCardanRepresentation : public PartRepresentation {
public:
	PassiveCardanRepresentation(std::string id, int orientation);
	virtual ~PassiveCardanRepresentation();
	virtual int arity();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* PASSIVECARDANREPRESENTATION_H_ */
