/*
 * PassiveWheelRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef PASSIVEWHEELREPRESENTATION_H_
#define PASSIVEWHEELREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class PassiveWheelRepresentation : public PartRepresentation {
public:
	PassiveWheelRepresentation(std::string id, int orientation, double radius);
	virtual ~PassiveWheelRepresentation();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
private:
	/**
	 * Radius of the wheel
	 */
	double radius_;

};

} /* namespace robogen */
#endif /* PASSIVEWHEELREPRESENTATION_H_ */
