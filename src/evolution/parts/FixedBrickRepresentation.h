/*
 * FixedBrickRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef FIXEDBRICKREPRESENTATION_H_
#define FIXEDBRICKREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class FixedBrickRepresentation : public PartRepresentation {
public:
	FixedBrickRepresentation(std::string id, int orientation);
	virtual ~FixedBrickRepresentation();
	virtual int arity();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
};

} /* namespace robogen */
#endif /* FIXEDBRICKREPRESENTATION_H_ */
