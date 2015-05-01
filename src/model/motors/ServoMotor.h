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
	static const float DEFAULT_MAX_FORCE_ROTATIONAL;
	static const float DEFAULT_MAX_FORCE_SERVO;
	static const float MIN_POS_RAD;
	static const float MAX_POS_RAD;
	static const float MIN_VELOCITY;
	static const float MAX_VELOCITY;

	/**
	 * Apply the motor to the provided joint. Initializes a servo controlled in velocity.
	 *
	 * @param joint ODE joint
	 * @param maxForce maximum force the motor can produce
	 * @param maxDirectionShiftsPerSecond, used for testing motor burn out
	 * 			default is -1, which means no motor burnout is tested
	 */
	ServoMotor(dJointID joint, float maxForce,
			ioPair id, int maxDirectionShiftsPerSecond=-1);

	/**
	 * Apply the motor to the provided joint. Initializes a servo controlled in position.
	 *
	 * @param joint ODE joint
	 * @param maxForce maximum force the motor can produce
	 * @param proportional control gain
	 * @param maxDirectionShiftsPerSecond, used for testing motor burn out
	 * 			default is -1, which means no motor burnout is tested
	 */
	ServoMotor(dJointID joint, float maxForce, float gain,
			ioPair id, int maxDirectionShiftsPerSecond=-1);

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
	void setPosition(float position, float step);

	/**
	 * Set the velocity of the motor in [0,1]
	 */
	void setVelocity(float velocity, float step);

	/**
	 * Set the maxDirectionShifts per second for testing motor burn out
	 * without an argument, will be set to -1 = disabled
	 */
	void setMaxDirectionShiftsPerSecond(int maxDirectionShiftsPerSecond=-1);

	/**
	 * @return true if the motor is driven in velocity
	 */
	bool isVelocityDriven();

	/**
	 * @return whether the motor is burnt out or not
	 */
	bool isBurntOut();

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

	/**
	 * Count number of time actuated
	 */
	unsigned int internalCounter_;


	/**
	 * Keep track of previous velocities,
	 * to be used for preventing burnout
	 */
	std::vector<float> previousVelocities_;

	bool isBurntOut_;

	int maxDirectionShiftsPerSecond_;

	void testBurnout(float velocity, float step);

};

}


#endif /* ROBOGEN_SERVO_MOTOR_H_ */
