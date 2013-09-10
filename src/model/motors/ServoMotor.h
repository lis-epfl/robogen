/*
 * @(#) ServoMotor.h   1.0   Feb 20, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch),
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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
#ifndef ROBOGEN_SERVO_MOTOR_H_
#define ROBOGEN_SERVO_MOTOR_H_

#include "model/motors/Motor.h"
#include "Robogen.h"

namespace robogen {

/**
 * A Servo motor
 */
class ServoMotor : public Motor {

public:

	static const float DEFAULT_GAIN;
	static const float DEFAULT_MAX_FORCE;
	static const float MIN_POS_RAD;
	static const float MAX_POS_RAD;
	static const float MIN_VELOCITY;
	static const float MAX_VELOCITY;

	/**
	 * Apply the motor to the provided joint. Initializes a servo controlled in velocity.
	 *
	 * @param joint ODE joint
	 * @param maxForce maximum force the motor can produce
	 */
	ServoMotor(dJointID joint, float maxForce,
			NeuralNetworkRepresentation::ioPair id);

	/**
	 * Apply the motor to the provided joint. Initializes a servo controlled in position.
	 *
	 * @param joint ODE joint
	 * @param maxForce maximum force the motor can produce
	 * @param proportional control gain
	 */
	ServoMotor(dJointID joint, float maxForce, float gain,
			NeuralNetworkRepresentation::ioPair id);

	/**
	 * Destructor
	 */
	virtual ~ServoMotor();

	/**
	 * Control the motor using its standard control modality (position or velocity)
	 */
	void control();

	/**
	 * Apply PI control to the motor to reach the desired position in [0,1]
	 */
	void setPosition(float position);

	/**
	 * Set the velocity of the motor in [0,1]
	 */
	void setVelocity(float velocity);

	/**
	 * @return true if the motor is driven in velocity
	 */
	bool isVelocityDriven();


private:

	/**
	 * ODE joint that models the servo
	 */
	dJointID joint_;

	/**
	 * Max force that the motor can produce
	 */
	float maxForce_;

	/**
	 * Gain for position control
	 */
	float gain_;

	/**
	 * True for velocity driven motors, false otherwise
	 */
	bool isVelocityDriven_;

};

}


#endif /* ROBOGEN_SERVO_MOTOR_H_ */
