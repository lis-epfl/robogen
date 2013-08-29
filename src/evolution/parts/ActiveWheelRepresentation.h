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
	ActiveWheelRepresentation(std::string id, int orientation, double radius);
	virtual ~ActiveWheelRepresentation();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
private:
	/**
	 * Radius of the wheel
	 */
	double radius_;
};

} /* namespace robogen */
#endif /* ACTIVEWHEELREPRESENTATION_H_ */
