/*
 * @(#) TouchSensor.cpp   1.0   Feb 27, 2013
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
#include "model/sensors/TouchSensor.h"

#include <map>

namespace robogen {

TouchSensor::TouchSensor(dSpaceID odeSpace, dGeomID sensorGeometry) :
		odeSpace_(odeSpace), sensorGeometry_(sensorGeometry) {

	this->sensorInfo_ = new CustomGeomData(CustomGeomData::TOUCH_SENSOR_INFO,
			new TouchSensorInfo());

	dGeomSetData(sensorGeometry, sensorInfo_);

}

TouchSensor::~TouchSensor() {
	delete (TouchSensorInfo*) sensorInfo_->data_;
	delete sensorInfo_;
}

bool TouchSensor::read() {

	// Main logic of the touch sensor is implemented in the main collide callback function, for
	// performance reasons, which handles the sensorInfo_ data struct
	return ((TouchSensorInfo*)sensorInfo_->data_)->touching;

}

void TouchSensor::reset() {
	((TouchSensorInfo*)sensorInfo_->data_)->touching = false;
}

}
