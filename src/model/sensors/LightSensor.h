/*
 * @(#) LightSensor.h   1.0   Feb 25, 2013
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
#ifndef ROBOGEN_LIGHT_SENSOR_H_
#define ROBOGEN_LIGHT_SENSOR_H_

#include <boost/shared_ptr.hpp>
#include <vector>

#include "Robogen.h"
#include "model/sensors/Sensor.h"
#include "model/objects/LightSource.h"

namespace robogen {

class LightSensor: public Sensor {

public:

	/**
	 * Default timestep for updating sensor value.
	 * This value is not used by the LightSensor class itself, but can used as a reference
	 * for a caller of LightSensor::read to set the updateSensor flag appropriately
	 */
	static const float DEFAULT_SENSOR_UPDATE_TIMESTEP;

	/**
	 * Minimum and maximum intensity value that is provided
	 * as output of the sensor
	 */
	static const float MIN_INTENSITY_VALUE;
	static const float MAX_INTENSITY_VALUE;

	/**
	 * Minimum and maximum intensity value mapped to the
	 * sensor outputs
	 */
	static const double MIN_INTENSITY;
	static const double MAX_INTENSITY;

	/**
	 * Half aperture, in degrees
	 */
	static const double HALF_APERTURE;

	/**
	 * Angular displacement for each ray tracing, in degrees
	 */
	static const double SENSOR_RESOLUTION;

	/**
	 * Initializes a light sensor
	 */
	LightSensor(dSpaceID odeSpace, std::vector<dBodyID> sensorBodies);

	/**
	 * Destructor
	 */
	virtual ~LightSensor();

	/**
	 * Update the light sensor
	 */
	void update(const osg::Vec3& position, const osg::Quat& attitude);

	/**
	 * Callback for collision handling between rays and the ODE space
	 */
	static void collisionCallback(void *data, dGeomID o1, dGeomID o2);

	/**
	 * Read sensor output, providing the light sources in the environment
	 * @lightSources
	 * @ambientLight
	 */
	float read(const std::vector<boost::shared_ptr<LightSource> >&
			lightSources, double ambientLight);

private:
	/**
	 * Calculates light intensity from angle and distance. This is the function
	 * to be modified according to light sensor calibration.
	 * @param angle
	 * @param distance
	 * @return light intensity
	 */
	static double lightIntensity(double angle, double distance);

	/**
	 * Position of the light sensor
	 */
	osg::Vec3 position_;

	/**
	 * Attitude of the light sensor
	 */
	osg::Quat attitude_;

	/**
	 * Ode collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * Space for rays
	 */
	dSpaceID raySpace_;

	/**
	 * Output of the last read
	 */
	float lastReadOutput_;

	/**
	 * ODE bodies of the light sensor
	 */
	std::vector<dGeomID> sensorGeoms_;

};

}

#endif /* ROBOGEN_LIGHT_SENSOR_H_ */
