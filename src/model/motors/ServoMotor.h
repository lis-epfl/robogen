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
#include "model/Joint.h"

namespace robogen {

/**
 * A Servo motor
 */
class ServoMotor : public Motor {

public:

	static const float DEFAULT_GAIN;
	static const float DEFAULT_MAX_FORCE_SERVO;
	static const float MIN_POS_RAD;
	static const float MAX_POS_RAD;
	static const float MIN_VELOCITY;
	static const float MAX_VELOCITY;

	/**
	 * Apply the motor to the provided joint. Initializes a servo controlled in position.
	 *
	 * @param id id of the motor
	 * @param joint Pointer to joint container
	 * @param maxForce maximum force the motor can produce
	 * @param proportional control gain
	 * @param maxDirectionShiftsPerSecond, used for testing motor burn out
	 * 			default is -1, which means no motor burnout is tested
	 */
	ServoMotor(ioPair id, boost::shared_ptr<Joint> joint, float maxForce,
			float gain, int maxDirectionShiftsPerSecond=-1);

	/**
	 * Destructor
	 */
	virtual ~ServoMotor();

	/**
	 * Apply PI control to the motor to reach the desired position in [0,1]
	 *
	 * @param position desired position of motor in [0,1]
	 * @param stepSize the stepSize of actuation
	 * 			(NOTE: not the physics stepSize)
	 */
	void setDesiredPosition(float position, float stepSize);

	inline void setGain(float gain) { gain_ = gain;	}

	/**
	 * step the motor using the latest control signal
	 *
	 * @param stepSize the physics stepSize
	*/
	virtual void step(float stepSize);

private:


	/**
	 * Gain for position control
	 */
	float gain_;

	float desiredPosition_;

	inline virtual float getSignal() { return desiredPosition_; }

	virtual int getNumDirectionFlips();
};

}


#endif /* ROBOGEN_SERVO_MOTOR_H_ */
