/*
 * @(#) ImuSensor.cpp   1.0   Mar 6, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Titus Cieslewski, Joshua Auerbach
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

#include <cmath>
#include "model/sensors/ImuSensor.h"
#include "model/sensors/Sensor.h"
#include "utils/RobogenUtils.h"

namespace robogen {

ImuSensor::ImuSensor(std::string partId) :
		initialized_(false) {
	std::string labels[] = { "x-acceleration", "y-acceleration",
			"z-acceleration", "Roll", "Pitch", "Yaw" };
	for (unsigned int i = 0; i < 6; ++i) {
		sensors_.push_back(
				boost::shared_ptr<Sensor>(
						new ImuSensorElement(partId + "-" + labels[i])));
	}
}

ImuSensor::~ImuSensor() {

}

void ImuSensor::update(const osg::Vec3& position, const osg::Quat& attitude,
		float timeElapsed, const osg::Vec3& gravity) {

	if (!initialized_) {
		position_ = position;
		attitude_ = attitude;
		velocity_ = osg::Vec3(0, 0, 0);
		acceleration_ = osg::Vec3(0, 0, 0);
		rotVelocity_ = osg::Vec3(0, 0, 0);
		initialized_ = true;
	}

	// =======================
	// 1. get attitude-agnostic IMU values (gyro knows attitude!)
	// =======================
	// position
	osg::Vec3 dPos = position - position_;
	position_ = position;
	// velocity & acceleration
	osg::Vec3 velocity = dPos / timeElapsed;
	acceleration_ = (velocity - velocity_) / timeElapsed;
	// subtract gravity from acceleration
	// in standard case this means subtracting -9.81 (i.e. adding 9.81) to z
	// (Positive, same effect as if accelerating up)
	acceleration_ -= gravity;
	velocity_ = velocity;
	// rotation
	double angle;
	osg::Vec3 rotaxis;
	osg::Quat dAttitude = attitude / attitude_; // this is attitude_.inverse() * attitude
	// in particular, no need to project!
	attitude_ = attitude;
	dAttitude.getRotate(angle, rotaxis);
	rotaxis.normalize(); // should be the case, but let's be safe


	/*
	 * We were having problems with sign switches in the quaternions
	 * leading to thinking that we had huge changes of orientations
	 * So we just assume that we never are rotating more than PI radians
	 * in one time step, and adjust the angle accordingly
	 */

	while (angle > M_PI) {
		angle -= (2*M_PI);
	}

	while (angle < (-M_PI)) {
		angle += (2*M_PI);
	}

	rotVelocity_ = rotaxis * angle / timeElapsed;


	// =======================
	// 2. create unit vectors for IMU reference system
	// =======================
	osg::Vec3 imuRef[3];
	for (int i = 0; i < 3; i++) {
		imuRef[i] = osg::Vec3((i == 0) ? 1 : 0, (i == 1) ? 1 : 0,
				(i == 2) ? 1 : 0);
		imuRef[i] = attitude * imuRef[i];
	}

	// =======================
	// 3. project accelerations onto IMU reference system
	// =======================
	osg::Vec3 accVal;
	for (int ref = 0; ref < 3; ref++) {
		// imuRef is unit, so simply scalar product
		accVal[ref] = acceleration_ * imuRef[ref];
	}

	sensors_[0]->updateValue(accVal.x());
	sensors_[1]->updateValue(accVal.y());
	sensors_[2]->updateValue(accVal.z());
	sensors_[3]->updateValue(rotVelocity_.x());
	sensors_[4]->updateValue(rotVelocity_.y());
	sensors_[5]->updateValue(rotVelocity_.z());

}


}

