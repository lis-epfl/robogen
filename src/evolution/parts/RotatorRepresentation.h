/*
 * RotatorRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef ROTATORREPRESENTATION_H_
#define ROTATORREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class RotatorRepresentation: public robogen::PartRepresentation {
public:
	RotatorRepresentation(std::string id, int orientation);
	virtual ~RotatorRepresentation();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* ROTATORREPRESENTATION_H_ */
