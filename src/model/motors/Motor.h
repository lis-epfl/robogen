/*
 * @(#) Motor.h   1.0   Feb 20, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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

#ifndef ROBOGEN_MOTOR_H_
#define ROBOGEN_MOTOR_H_

#include "evolution/representation/NeuralNetworkRepresentation.h"
#include "model/Joint.h"

namespace robogen {

class Motor {
	
public:
	
	/**
	 * constructor
	 *
	 * @param id id of the motor
	 * @param joint pointer to the joint container
	 * @param maxForce the maximum force the motor can produce
	 * @param maxDirectionShiftsPerSecond, used for testing motor burn out
	 *			default is -1, which means no motor burnout is tested
	*/
	Motor(ioPair id, boost::shared_ptr<Joint> joint, float maxForce,
			int maxDirectionShiftsPerSecond=-1);

	/**
	 * destructor
	 *
	*/
	virtual ~Motor() {}

	/**
	 * get the id of this motor
	 *
	 * @param stepSize the physics stepSize
	*/
	ioPair getId();

	/**
	 * Set the maxDirectionShifts per second for testing motor burn out
	 * without an argument, will be set to -1 = disabled
	 *
	 * @param maxDirectionShiftsPerSecond max number of direction shifts to
	 * 			tolerate per second of simulated time
	 */
	void setMaxDirectionShiftsPerSecond(int maxDirectionShiftsPerSecond=-1);


	/**
	 * step the motor using the latest control signal
	 *
	 * @param stepSize the physics stepSize
	*/
	virtual void step(float stepSize) = 0;



	/**
	 * @return whether the motor is burnt out or not
	 */
	bool isBurntOut();

	dReal getTorque();
	dReal getVelocity();
	dReal getPosition();


protected:

	ioPair id_;

	/**
	 * Keep track of previous motor signals,
	 * to be used for preventing burnout
	 */
	std::vector<float> previousSignals_;


	template <typename T> int sgn(T val) {
	    return (T(0) < val) - (val < T(0));
	}

	/**
	 * Pointer to container of ODE joint that models the motor
	 */
	boost::shared_ptr<Joint> joint_;

	bool isBurntOut_;

	bool shouldStep_;

	void testBurnout(float stepSize);

	int maxDirectionShiftsPerSecond_;

	/**
	 * Max force that the motor can produce
	 */
	float maxForce_;

	/**
	 * Count number of time actuated
	 */
	unsigned int internalCounter_;

	dJointFeedback  fback_;

	virtual float getSignal() = 0;
	virtual int getNumDirectionFlips() = 0;

};

}


#endif /* ROBOGEN_MOTOR_H_ */
