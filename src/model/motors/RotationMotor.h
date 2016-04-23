/*
 * @(#) RotationMotor.h   1.0   Mar 3, 2016
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_ROTATIONMOTOR_H_
#define ROBOGEN_ROTATIONMOTOR_H_


#include "model/motors/Motor.h"
#include "Robogen.h"


namespace robogen {

/**
 * A Rotation motor
 */
class RotationMotor : public Motor {

public:

	static const float DEFAULT_MAX_FORCE_ROTATIONAL;
	static const float MIN_VELOCITY;
	static const float MAX_VELOCITY;

	/**
	 * Apply the motor to the provided joint. Initializes a velocity
	 * controlled rotation motor
	 *
	 * @param id id of the motor
	 * @param joint Pointer to joint container
	 * @param maxForce maximum force the motor can produce
	 * @param maxDirectionShiftsPerSecond, used for testing motor burn out
	 * 			default is -1, which means no motor burnout is tested
	 */
	RotationMotor(ioPair id, boost::shared_ptr<Joint> joint,
			float maxForce, int maxDirectionShiftsPerSecond=-1);

	/**
	 * Destructor
	 */
	virtual ~RotationMotor();

	/**
	 * Set the velocity of the motor in [0,1]
	 *
	 * @param velocity desired velocity of motor in [0,1]
	 * @param stepSize the stepSize of actuation
	 * 			(NOTE: not the physics stepSize)
	 */
	void setDesiredVelocity(float velocity, float stepSize);

	/**
	 * step the motor using the latest control signal
	 *
	 * @param stepSize the physics stepSize
	*/
	virtual void step(float stepSize);

	inline virtual float getSignal() { return desiredVelocity_; }

	virtual int getNumDirectionFlips();

private:

	float desiredVelocity_;

};

}




#endif /* ROBOGEN_ROTATIONMOTOR_H_ */
