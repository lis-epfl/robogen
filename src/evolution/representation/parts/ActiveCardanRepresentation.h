/*
 * ActiveCardanRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef ACTIVECARDANREPRESENTATION_H_
#define ACTIVECARDANREPRESENTATION_H_

#include "evolution/representation/PartRepresentation.h"

namespace robogen {

class ActiveCardanRepresentation : public PartRepresentation {
public:
	ActiveCardanRepresentation(std::string id, int orientation);
	virtual ~ActiveCardanRepresentation();
	virtual boost::shared_ptr<PartRepresentation> cloneSubtree();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* ACTIVECARDANREPRESENTATION_H_ */
