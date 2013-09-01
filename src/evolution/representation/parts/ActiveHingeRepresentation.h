/*
 * ActiveHingeRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef ACTIVEHINGEREPRESENTATION_H_
#define ACTIVEHINGEREPRESENTATION_H_

#include "evolution/representation/PartRepresentation.h"

namespace robogen {

class ActiveHingeRepresentation : public PartRepresentation {
public:
	ActiveHingeRepresentation(std::string id, int orientation);
	virtual ~ActiveHingeRepresentation();
	virtual boost::shared_ptr<PartRepresentation> cloneSubtree();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* ACTIVEHINGEREPRESENTATION_H_ */
