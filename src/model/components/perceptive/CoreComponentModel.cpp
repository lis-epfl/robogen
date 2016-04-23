/*
 * @(#) CoreComponentModel.cpp   1.0   Feb 8, 2013
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
#include "model/components/perceptive/CoreComponentModel.h"

namespace robogen {

// mass of just the brick
const float CoreComponentModel::BRICK_MASS = inGrams(10.2);//inGrams(14.9);
// mass of brick with electronics (including battery)
const float CoreComponentModel::CORE_MASS = inGrams(10.2 + 34.3);//inGrams(14.9 + 40.5);
const float CoreComponentModel::HEIGHT = inMm(35.5);
const float CoreComponentModel::WIDTH = inMm(41);//inMm(46.5);
const float CoreComponentModel::SLOT_THICKNESS = inMm(1.5);

CoreComponentModel::CoreComponentModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id, bool isCore, bool hasSensors) :
		PerceptiveComponent(odeWorld, odeSpace, id), isCore_(isCore),
		hasSensors_(hasSensors) {

	if (hasSensors) {
		sensor_.reset(new ImuSensor(id + "-IMU"));
	}

}

CoreComponentModel::~CoreComponentModel() {

}

bool CoreComponentModel::initModel() {

	coreComponent_ = this->addBox(isCore_ ? CORE_MASS : BRICK_MASS,
			osg::Vec3(0, 0, 0), WIDTH, WIDTH,
			HEIGHT, B_CORE_COMPONENT_ID);

	return true;
}

boost::shared_ptr<SimpleBody> CoreComponentModel::getRoot() {
	return coreComponent_;
}

boost::shared_ptr<SimpleBody> CoreComponentModel::getSlot(unsigned int /*i*/) {
	return coreComponent_;
}

osg::Vec3 CoreComponentModel::getSlotPosition(unsigned int i) {

	if (i >= NUM_SLOTS) {
		std::cout << "[CoreComponentModel] Invalid slot: " << i << std::endl;
		assert(i < NUM_SLOTS);
	}

	osg::Vec3 curPos = this->getRootPosition();
	// want the slot to be the coordinates specifying the edge of where the
	// adjoining part touches
	osg::Vec3 slotAxis = this->getSlotAxis(i) *
			(WIDTH / 2 - SLOT_THICKNESS);
	curPos = curPos + slotAxis;

	return curPos;

}

osg::Vec3 CoreComponentModel::getSlotAxis(unsigned int i) {

	if (i >= NUM_SLOTS) {
		std::cout << "[CoreComponentModel] Invalid slot: " << i << std::endl;
		assert(i < NUM_SLOTS);
	}

	osg::Quat quat = this->getRootAttitude();
	osg::Vec3 axis;

	if (i == LEFT_FACE_SLOT) {

		// Left face
		axis.set(-1, 0, 0);

	} else if (i == RIGHT_FACE_SLOT) {

		// Right face
		axis.set(1, 0, 0);
	} else if (i == FRONT_FACE_SLOT) {

		// Front face
		axis.set(0, -1, 0);

	} else if (i == BACK_FACE_SLOT) {

		// Back face
		axis.set(0, 1, 0);

	}
#ifndef ENFORCE_PLANAR
	else if (i == TOP_FACE_SLOT) {

		// Top face
		axis.set(0, 0, 1);

	} else if (i == BOTTOM_FACE_SLOT) {

		// Bottom face
		axis.set(0, 0, -1);
	}
#endif


	return quat * axis;

}

osg::Vec3 CoreComponentModel::getSlotOrientation(unsigned int i) {

	if (i >= NUM_SLOTS) {
		std::cout << "[CoreComponentModel] Invalid slot: " << i << std::endl;
		assert(i < NUM_SLOTS);
	}

	osg::Quat quat = this->getRootAttitude();
	osg::Vec3 tangent;

	if (i == LEFT_FACE_SLOT) {

		// Left face
		tangent.set(0, 1, 0);

	} else if (i == RIGHT_FACE_SLOT) {

		// Right face
		tangent.set(0, 1, 0);
	} else if (i == FRONT_FACE_SLOT) {

		// Front face
		tangent.set(0, 0, 1);

	} else if (i == BACK_FACE_SLOT) {

		// Back face
		tangent.set(0, 0, 1);

	}
#ifndef ENFORCE_PLANAR
	else if (i == TOP_FACE_SLOT) {

		// Top face
		tangent.set(1, 0, 0);

	} else if (i == BOTTOM_FACE_SLOT) {

		// Bottom face
		tangent.set(1, 0, 0);
	}
#endif

	return quat * tangent;

}

void CoreComponentModel::getSensors(
		std::vector<boost::shared_ptr<Sensor> >& sensors) {
	if (sensor_ != NULL) {
		sensor_->getSensors(sensors);
	}
}

void CoreComponentModel::updateSensors(boost::shared_ptr<Environment>& env) {
	if (sensor_ != NULL) {
		dVector3 gravity;
		dWorldGetGravity(getPhysicsWorld(), gravity);
		sensor_->update(this->getRootPosition(), this->getRootAttitude(),
				env->getTimeElapsed(),
				osg::Vec3(gravity[0], gravity[1], gravity[2]));
	}
}

}
