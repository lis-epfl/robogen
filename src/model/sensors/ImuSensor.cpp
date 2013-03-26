/*
 * ImuSensor.cpp
 *
 *  Created on: Mar 6, 2013
 *      Author: andrea
 */

#include <cmath>
#include "model/sensors/ImuSensor.h"
#include "model/sensors/SimpleSensor.h"
#include "utils/RobogenUtils.h"

namespace robogen {

ImuSensor::ImuSensor() {
	for (unsigned int i = 0; i < 6; ++i) {
		sensors_.push_back(boost::shared_ptr<SimpleSensor>(new SimpleSensor()));
	}
}

ImuSensor::~ImuSensor() {

}

void ImuSensor::update(const osg::Vec3& position, const osg::Quat& attitude,
		float timeElapsed) {

	if (!initialized_) {

		position_ = position;
		attitude_ = attitude;
		velocity_ = osg::Vec3(0, 0, 0);
		acceleration_ = osg::Vec3(0, 0, 0);
		rotVelocity_ = osg::Vec3(0, 0, 0);
		initialized_ = true;
	}

	osg::Vec3 dPos = position - position_;
	position_ = position;

	osg::Vec3 velocity = dPos * timeElapsed;
	acceleration_ = (velocity - velocity_) * timeElapsed;
	velocity_ = velocity;

	float roll, pitch, yaw;
	RobogenUtils::getAngle(attitude, attitude_, roll, pitch, yaw);

	osg::Vec3 dAngle(roll, pitch, yaw);
	dAngle *= timeElapsed;

	rotVelocity_ = dAngle;

	sensors_[0]->update(acceleration_.x());
	sensors_[1]->update(acceleration_.y());
	sensors_[2]->update(acceleration_.z());
	sensors_[3]->update(rotVelocity_.x());
	sensors_[4]->update(rotVelocity_.y());
	sensors_[5]->update(rotVelocity_.z());

}

void ImuSensor::getSensors(std::vector<boost::shared_ptr<Sensor> >& sensors) {
	sensors.clear();
	sensors.insert(sensors.begin(), sensors_.begin(), sensors_.end());
}

}

