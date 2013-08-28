/*
 * CoreComponentRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef CORECOMPONENTREPRESENTATION_H_
#define CORECOMPONENTREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class CoreComponentRepresentation : public PartRepresentation {
public:
	CoreComponentRepresentation(std::string id, int orientation);
	virtual ~CoreComponentRepresentation();
	virtual int arity();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* CORECOMPONENTREPRESENTATION_H_ */
