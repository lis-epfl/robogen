/*
 * @(#) QBindings.cpp   1.0   Dec 23, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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
#ifdef QT5_ENABLED

#include "QBindings.h"
#include "utils/QScriptUtils.h"

#include "model/sensors/Sensor.h"
#include "utils/RobogenUtils.h"

namespace robogen {
namespace qscript {


// QMotor

QMotor::QMotor(boost::weak_ptr<Motor> basePtr) : basePtr_(basePtr) { }

QScriptValue QMotor::getId() {
	if(!id_.isValid()) {
		id_ = engine()->newObject();
		ioPair id = basePtr_.lock()->getId();
		id_.setProperty("partId", QString::fromStdString(id.first));
		id_.setProperty("ioId", id.second);
	}
	return id_;
}

QScriptValue QMotor::getVelocity() {
	return basePtr_.lock()->getVelocity();
}
QScriptValue QMotor::getPosition() {
	return basePtr_.lock()->getPosition();
}
QScriptValue QMotor::getTorque() {
	return basePtr_.lock()->getTorque();
}

// QSensor

QSensor::QSensor(boost::weak_ptr<Sensor> basePtr) : basePtr_(basePtr) {}

QScriptValue QSensor::getLabel() {
	return QString::fromStdString(basePtr_.lock()->getLabel());
}

QScriptValue QSensor::read() {
	return basePtr_.lock()->read();
}

// QModel

QModel::QModel(boost::weak_ptr<Model> basePtr) : basePtr_(basePtr) {

}

QScriptValue QModel::getRootPosition() {
	return valFromVec3(engine(),
			basePtr_.lock()->getRootPosition());
}

QScriptValue QModel::getRootAttitude() {
	return valFromQuat(engine(),
			basePtr_.lock()->getRootAttitude());
}

QScriptValue QModel::getType() {
	return QString::fromStdString(RobogenUtils::getPartType(basePtr_.lock()));
}


// QRobot

QRobot::QRobot(boost::weak_ptr<Robot> basePtr) : basePtr_(basePtr) {

}

QScriptValue QRobot::getCoreComponent() {
	 if(!coreComponent_.isValid()) {
		 coreComponent_ = engine()->newQObject(
				 new QModel(basePtr_.lock()->getCoreComponent()),
				 QScriptEngine::ScriptOwnership);
	 }
	 return coreComponent_;
}

QScriptValue QRobot::getBodyParts() {
	std::vector<boost::shared_ptr<Model> > bodyParts = basePtr_.lock()->getBodyParts();

	if (!bodyParts_.isValid()) {
		bodyParts_ = engine()->newArray(bodyParts.size());
		for(size_t i = 0; i<bodyParts.size(); ++i) {
			bodyParts_.setProperty(i,  engine()->newQObject(
					new QModel(bodyParts[i]),
					QScriptEngine::ScriptOwnership));
		}
	}
	return bodyParts_;
}

QScriptValue QRobot::getMotors() {
	std::vector<boost::shared_ptr<Motor> > motors = basePtr_.lock()->getMotors();

	if (!motors_.isValid()) {
		motors_ = engine()->newArray(motors.size());
		for(size_t i = 0; i<motors.size(); ++i) {
			QMotor* motor = new QMotor(motors[i]);
			motors_.setProperty(i,  engine()->newQObject(motor,
					QScriptEngine::ScriptOwnership));
		}
	}
	return motors_;
}

QScriptValue QRobot::getSensors() {
	std::vector<boost::shared_ptr<Sensor> > sensors = basePtr_.lock()->getSensors();

	if (!sensors_.isValid()) {
		sensors_ = engine()->newArray(sensors.size());
		for(size_t i = 0; i<sensors.size(); ++i) {
			sensors_.setProperty(i,  engine()->newQObject(
					new QSensor(sensors[i]),
					QScriptEngine::ScriptOwnership));
		}
	}
	return sensors_;
}

/*QScriptValue QRobot::getPosition() {
	return valFromVec3(engine_.lock(),
			robot_.lock()->getCoreComponent()->getRootPosition());
}*/

// QPositionObservable
QPositionObservable::QPositionObservable(
		boost::weak_ptr<PositionObservable> basePtr) : basePtr_(basePtr) {}


QScriptValue QPositionObservable::getPosition() {
	return valFromVec3(engine(), basePtr_.lock()->getPosition());
}

QScriptValue QPositionObservable::getAttitude() {
	return valFromQuat(engine(), basePtr_.lock()->getAttitude());
}

// QLightSource

QScriptValue QLightSource::getIntensity() {
	return boost::dynamic_pointer_cast<LightSource>(basePtr_.lock()
			)->getIntensity();
}

void QLightSource::setPosition(float x, float y, float z) {
	boost::dynamic_pointer_cast<LightSource>(basePtr_.lock()
				)->setPosition(osg::Vec3(x,y,z));
}

void QLightSource::setIntensity(float intensity) {
	boost::dynamic_pointer_cast<LightSource>(basePtr_.lock()
				)->setIntensity(intensity);
}

// QBoxObstacle

QScriptValue QBoxObstacle::getSize() {
	return valFromVec3(engine(), boost::dynamic_pointer_cast<BoxObstacle>(
			basePtr_.lock())->getSize());
}

// QEnvironment
QEnvironment::QEnvironment(boost::weak_ptr<Environment> basePtr) :
		basePtr_(basePtr) {}

QScriptValue QEnvironment:: getLightSources() {
	std::vector<boost::shared_ptr<LightSource> > lightSources =
			basePtr_.lock()->getLightSources();

	if (!lightSources_.isValid()) {
		lightSources_ = engine()->newArray(lightSources.size());
		for(size_t i = 0; i<lightSources.size(); ++i) {
			lightSources_.setProperty(i,  engine()->newQObject(
					new QLightSource(lightSources[i]),
					QScriptEngine::ScriptOwnership));
		}
	}
	return lightSources_;
}

QScriptValue QEnvironment:: getAmbientLight() {
	return basePtr_.lock()->getAmbientLight();
}

QScriptValue QEnvironment:: getObstacles() {
	std::vector<boost::shared_ptr<Obstacle> > obstacles =
			basePtr_.lock()->getObstacles();

	if (!obstacles_.isValid()) {
		obstacles_ = engine()->newArray(obstacles.size());
		for(size_t i = 0; i<obstacles.size(); ++i) {
			boost::shared_ptr<BoxObstacle> box =
					boost::dynamic_pointer_cast<BoxObstacle>(obstacles[i]);
			QObstacle* obstacle;
			if (box)
				obstacle = new QBoxObstacle(box);
			else
				obstacle = new QObstacle(obstacles[i]);

			obstacles_.setProperty(i,  engine()->newQObject(
					obstacle,QScriptEngine::ScriptOwnership));
		}
	}
	return obstacles_;
}




}
}
#endif
