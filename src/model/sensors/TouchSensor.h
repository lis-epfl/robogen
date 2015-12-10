/*
 * @(#) TouchSensor.h   1.0   Feb 27, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#include <string>

#include "Robogen.h"
#include "model/sensors/Sensor.h"
#include "model/SimpleBody.h"

namespace robogen {

class TouchSensor: public Sensor {

public:

	/**
	 * Initializes a touch sensor including its geometry
	 */
	TouchSensor(dSpaceID odeSpace, boost::shared_ptr<SimpleBody> body,
			std::string label);


	virtual ~TouchSensor();

	void update();

	/**
	 * Read sensor output
	 * @return true if the sensor is touching something, false otherwise
	 */
	bool read();

private:

	/**
	 * Ode collision callback
	 */
	static void collisionCallback(void *data, dGeomID o1, dGeomID o2);

	/**
	 * Ode collision space
	 */
	dSpaceID collideSpace_;

	/**
	 * Body representing this touch sensor
	 */
	boost::shared_ptr<SimpleBody> body_;


};

}

#endif /* ROBOGEN_TOUCH_SENSOR_H_ */
