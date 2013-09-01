/*
 * LightSensorRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef LIGHTSENSORREPRESENTATION_H_
#define LIGHTSENSORREPRESENTATION_H_

#include "evolution/representation/PartRepresentation.h"

namespace robogen {

class LightSensorRepresentation : public PartRepresentation {
public:
	LightSensorRepresentation(std::string id, int orientation);
	virtual ~LightSensorRepresentation();
	virtual boost::shared_ptr<PartRepresentation> cloneSubtree();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* LIGHTSENSORREPRESENTATION_H_ */
