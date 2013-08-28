/*
 * ParametricBarJointRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef PARAMETRICBARJOINTREPRESENTATION_H_
#define PARAMETRICBARJOINTREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class ParametricBarJointRepresentation : public PartRepresentation {
public:
	ParametricBarJointRepresentation(std::string id, int orientation);
	virtual ~ParametricBarJointRepresentation();
	virtual int arity();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* PARAMETRICBARJOINTREPRESENTATION_H_ */
