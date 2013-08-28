/*
 * ActiveWheelRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef ACTIVEWHEELREPRESENTATION_H_
#define ACTIVEWHEELREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class ActiveWheelRepresentation : public PartRepresentation {
public:
	ActiveWheelRepresentation(std::string id, int orientation);
	virtual ~ActiveWheelRepresentation();
	virtual int arity();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* ACTIVEWHEELREPRESENTATION_H_ */
