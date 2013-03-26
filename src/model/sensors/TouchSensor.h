/*
 * @(#) TouchSensor.h   1.0   Feb 27, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#ifndef ROBOGEN_TOUCH_SENSOR_H_
#define ROBOGEN_TOUCH_SENSOR_H_

#include <boost/shared_ptr.hpp>
#include <vector>

#include "Robogen.h"
#include "model/sensors/Sensor.h"

namespace robogen {

class TouchSensor: public Sensor {

public:

	class TouchSensorInfo {
	public:
		TouchSensorInfo() : touching(false) {
		}
		bool touching;
	};


	/**
	 * Initializes a touch sensor
	 */
	TouchSensor(dSpaceID odeSpace, dGeomID sensorGeometry);

	/**
	 * Destructor
	 */
	virtual ~TouchSensor();

	/**
	 * Read sensor output
	 * @return true if the sensor is touching something, false otherwise
	 */
	bool read();

	/**
	 * Resets the sensor. Must be called before each round of collision detection.
	 */
	void reset();

private:

	/**
	 * Ode collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * The geometry of the sensor
	 */
	dGeomID sensorGeometry_;

	/**
	 * Sensor info
	 */
	CustomGeomData* sensorInfo_;
};

}

#endif /* ROBOGEN_TOUCH_SENSOR_H_ */
