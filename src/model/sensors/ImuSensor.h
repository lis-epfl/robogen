/*
 * @(#) ImuSensor.h   1.0   Mar 6, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Titus Cieslewski, Joshua Auerbach
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
#ifndef ROBOGE_IMU_SENSOR_H_
#define ROBOGE_IMU_SENSOR_H_

#include <osg/Quat>
#include <osg/Vec3>
#include <vector>

#include "model/sensors/SensorGroup.h"

namespace robogen {

class ImuSensorElement : public Sensor {
public:
	inline ImuSensorElement(std::string label) : Sensor(label) {}
};

class ImuSensor: public SensorGroup {

public:

	/**
	 * Collection of sensors for accel + gyro
	 * When call getSensors will get a sensors for each acceleration outputs
	 * (acceleration along the three axis)
	 * and gyro outputs (+/- 360°) (rotational velocity)
	 * (x-acc, y-acc, z-acc, x-gryo, y-gyro, z-gryo)
	 */

	ImuSensor(std::string partId);

	virtual ~ImuSensor();

	/**
	 * Update the sensor position
	 * @param position
	 * @param timeElapsed time elapsed since last update
	 * @param gravity gravitational vector of environment
	 */
	void update(const osg::Vec3& position, const osg::Quat& attitude,
			float timeElapsed, const osg::Vec3& gravity);




private:

	/**
	 * Previous sensor position
	 */
	osg::Vec3 position_;

	/**
	 * Velocity
	 */
	osg::Vec3 velocity_;

	/**
	 * Velocity
	 */
	osg::Vec3 acceleration_;

	/**
	 * Attitude
	 */
	osg::Quat attitude_;

	/**
	 * Rotational velocity
	 */
	osg::Vec3 rotVelocity_;

	/**
	 * True after first call of update()
	 */
	bool initialized_;

};

}

#endif /* ROBOGE_IMU_SENSOR_H_ */
