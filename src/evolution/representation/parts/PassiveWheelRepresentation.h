/*
 * PassiveWheelRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef PASSIVEWHEELREPRESENTATION_H_
#define PASSIVEWHEELREPRESENTATION_H_

#include "evolution/representation/PartRepresentation.h"

namespace robogen {

class PassiveWheelRepresentation : public PartRepresentation {
public:
	PassiveWheelRepresentation(std::string id, int orientation, double radius);
	virtual ~PassiveWheelRepresentation();
	virtual boost::shared_ptr<PartRepresentation> cloneSubtree();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* PASSIVEWHEELREPRESENTATION_H_ */
