/*
 * TouchSensorRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef TOUCHSENSORREPRESENTATION_H_
#define TOUCHSENSORREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class TouchSensorRepresentation : public PartRepresentation {
public:
	TouchSensorRepresentation(std::string id, int orientation);
	virtual ~TouchSensorRepresentation();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* TOUCHSENSORREPRESENTATION_H_ */
