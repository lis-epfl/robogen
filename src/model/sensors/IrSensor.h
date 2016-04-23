/*
 * @(#) LightSensor.h   1.0   Oct 27, 2015
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
#ifndef ROBOGEN_IR_SENSOR_H_
#define ROBOGEN_IR_SENSOR_H_

#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

#include "Robogen.h"
#include "model/sensors/Sensor.h"
#include "model/SimpleBody.h"
#include "SensorGroup.h"

namespace robogen {



class IrSensorElement : public Sensor {
public:

	// N.B.  For now only the distance (IR) measurement is enabled
	// in the hardware, but leaving like this in case we enable the ambient
	// light sensing in the future
	enum Type {
		IR,
		AMBIENT_LIGHT
	};

	inline IrSensorElement(std::string baseLabel, Type type) :
			Sensor(baseLabel /*+ ((type == IR) ?  "-Infrared" :
										"-AmbientLight")*/),
			baseLabel_(baseLabel), type_(type) {
	}

	inline Type getType() { return type_; }
	inline const std::string &getBaseLabel() { return baseLabel_; }

private:
	std::string baseLabel_;
	Type type_;

};

class IrSensor: public SensorGroup {

public:

	static const float SENSOR_RANGE;

	/**
	 * Initializes a light sensor
	 */
	IrSensor(dSpaceID odeSpace,
			std::vector<boost::shared_ptr<SimpleBody> > sensorBodies,
			std::string baseLabel);

	/**
	 * Destructor
	 */
	virtual ~IrSensor();

	/**
	 * Update the light sensor
	 */
	void update(const osg::Vec3& position, const osg::Quat& attitude);

	/**
	 * Callback for collision handling between rays and the ODE space
	 */
	static void collisionCallback(void *data, dGeomID o1, dGeomID o2);

private:
	/**
	 * Calculates intensity at light sensor from angle, light's intensity,
	 * and distance. This is the function to be modified according to
	 * light sensor calibration.
	 * @param angle
	 * @param lightIntensity
	 * @param distance
	 * @return intensity
	 */
	static double getIntensity(double angle, double lightIntensity,
			double distance);

	/**
	 * Ode collision space
	 */
	dSpaceID odeSpace_;


	/**
	 * Position of the IR sensor
	 */
	osg::Vec3 position_;

	/**
	 * Attitude of the IR sensor
	 */
	osg::Quat attitude_;

	/**
	 * Space for rays
	 */
	dSpaceID raySpace_;

	/**
	 * Output of the last read
	 */
	float lastReadOutput_;

	/**
	 * ODE bodies of the IR sensor
	 */
	std::vector<dGeomID> sensorGeoms_;

};

}

#endif /* ROBOGEN_IR_SENSOR_H_ */
