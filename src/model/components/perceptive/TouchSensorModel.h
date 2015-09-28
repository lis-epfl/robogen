/*
 * @(#) TouchSensorModel.h   1.0   Feb 27, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#ifndef ROBOGEN_TOUCH_SENSOR_MODEL_H_
#define ROBOGEN_TOUCH_SENSOR_MODEL_H_

#include <boost/shared_ptr.hpp>
#include "model/PerceptiveComponent.h"
#include "model/sensors/TouchSensor.h"

namespace robogen {

/**
 * A Touch Sensor is modelled with 1 box for the base and two boxes for the left/right sensors
 */
class TouchSensorModel: public PerceptiveComponent {

public:

	static const float MASS;
	static const float SENSOR_BASE_WIDTH;
	static const float SENSOR_BASE_THICKNESS;
	static const float SENSOR_THICKNESS;
	static const float SENSOR_WIDTH;
	static const float SENSOR_HEIGHT;
	static const float SENSOR_SEPARATION;

	static const unsigned int SLOT_A = 0;

	static const unsigned int B_SENSOR_BASE_ID = 0;
	static const unsigned int B_SENSOR_LEFT = 1;
	static const unsigned int B_SENSOR_RIGHT = 2;

	TouchSensorModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id);

	virtual ~TouchSensorModel();

	virtual bool initModel();

	virtual boost::shared_ptr<SimpleBody> getRoot();

	virtual boost::shared_ptr<SimpleBody> getSlot(unsigned int i);

	virtual osg::Vec3 getSlotPosition(unsigned int i);

	virtual osg::Vec3 getSlotOrientation(unsigned int i);

	virtual osg::Vec3 getSlotAxis(unsigned int i);

	virtual void getSensors(std::vector<boost::shared_ptr<Sensor> >& sensors);

	virtual void updateSensors(boost::shared_ptr<Environment>& env);

private:

	boost::shared_ptr<SimpleBody> sensorRoot_;

	boost::shared_ptr<TouchSensor> sensorLeft_;
	boost::shared_ptr<TouchSensor> sensorRight_;

};

}

#endif /* ROBOGEN_TOUCH_SENSOR_MODEL_H_ */
