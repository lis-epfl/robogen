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
	ParametricBarJointRepresentation(std::string id, int orientation,
			double length, double inclination, double rotation);
	virtual ~ParametricBarJointRepresentation();
	virtual boost::shared_ptr<PartRepresentation> cloneSubtree();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
private:
	/**
	 * Length of the parametric bar joint
	 */
	double length_;

	/**
	 * Inclination angle of the parametric bar joint
	 */
	double inclination_;

	/**
	 * Rotation angle of the parametric bar joint
	 */
	double rotation_;
};

} /* namespace robogen */
#endif /* PARAMETRICBARJOINTREPRESENTATION_H_ */
