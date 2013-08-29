/*
 * PassiveHingeRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef PASSIVEHINGEREPRESENTATION_H_
#define PASSIVEHINGEREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class PassiveHingeRepresentation : public PartRepresentation {
public:
	PassiveHingeRepresentation(std::string id, int orientation);
	virtual ~PassiveHingeRepresentation();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* PASSIVEHINGEREPRESENTATION_H_ */
