/*
 * @(#) ActiveWheelModel.cpp   1.0   Feb 13, 2013
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
#include "model/components/actuated/ActiveWheelModel.h"
#include "model/motors/ServoMotor.h"

namespace robogen {

const float ActiveWheelModel::MASS_SLOT = inGrams(4);
const float ActiveWheelModel::MASS_SERVO = inGrams(9);
const float ActiveWheelModel::MASS_WHEEL = inGrams(5);

const float ActiveWheelModel::SLOT_WIDTH = inMm(34);
const float ActiveWheelModel::SLOT_THICKNESS = inMm(1.5);
const float ActiveWheelModel::SERVO_Z_OFFSET = inMm(9); // zCenter shift respect to slot z-center
const float ActiveWheelModel::SERVO_WIDTH = inMm(14);
const float ActiveWheelModel::SERVO_LENGTH = inMm(36.8);
const float ActiveWheelModel::SERVO_HEIGHT = inMm(14);
const float ActiveWheelModel::WHEEL_THICKNESS = inMm(3);

ActiveWheelModel::ActiveWheelModel(dWorldID odeWorld, dSpaceID odeSpace,
      std::string id, float radius) :
      ActuatedComponent(odeWorld, odeSpace, id), radius_(radius) {

}

ActiveWheelModel::~ActiveWheelModel() {

}

float ActiveWheelModel::getRadius() const {
   return radius_;
}

bool ActiveWheelModel::initModel() {

   // Create the 4 components of the hinge
   wheelRoot_ = this->createBody(B_SLOT_ID);
   dBodyID servo = this->createBody(B_SERVO_ID);
   dBodyID wheel = this->createBody(B_WHEEL_ID);

   // Set the masses for the various boxes
   dMass mass;

   float separation = inMm(0.1);

   this->createBoxGeom(wheelRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
         SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

   dReal xServo = SLOT_THICKNESS / 2 + separation + SERVO_LENGTH / 2;
   dReal zServo = -SLOT_WIDTH / 2 + SERVO_Z_OFFSET + SERVO_HEIGHT / 2;
   this->createBoxGeom(servo, MASS_SERVO, osg::Vec3(xServo, 0, zServo),
         SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT);

   dReal xWheel = xServo + SERVO_LENGTH / 2;
   this->createCylinderGeom(wheel, MASS_WHEEL, osg::Vec3(xWheel, 0, 0), 1,
         getRadius(), WHEEL_THICKNESS);

   // Create joints to hold pieces in position

   // slot <slider> hinge
   this->fixBodies(wheelRoot_, servo, osg::Vec3(1, 0, 0));

   // servo <(hinge)> wheel
   dJointID joint = dJointCreateHinge(this->getPhysicsWorld(), 0);
   dJointAttach(joint, servo, wheel);
   dJointSetHingeAxis(joint, 1, 0, 0);
   dJointSetHingeAnchor(joint, xWheel, 0, 0);

   // Create servo
   this->motor_.reset(
         new ServoMotor(joint, ServoMotor::DEFAULT_MAX_FORCE,
        		 ioPair(this->getId(),0)));

   return true;

}

dBodyID ActiveWheelModel::getRoot() {
   return wheelRoot_;
}

dBodyID ActiveWheelModel::getSlot(unsigned int i) {

   if (i > 1) {
      std::cout << "[ActiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   return wheelRoot_;
}

osg::Vec3 ActiveWheelModel::getSlotPosition(unsigned int i) {

   if (i > 1) {
      std::cout << "[ActiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Vec3 curPos = this->getPosition(wheelRoot_);
   osg::Vec3 slotAxis = this->getSlotAxis(i);
   return curPos + slotAxis * (SLOT_THICKNESS / 2);

}

osg::Vec3 ActiveWheelModel::getSlotAxis(unsigned int i) {

   if (i > 1) {
      std::cout << "[ActiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Quat quat = this->getAttitude(this->wheelRoot_);
   osg::Vec3 axis(-1, 0, 0);

   return quat * axis;

}

osg::Vec3 ActiveWheelModel::getSlotOrientation(unsigned int i) {

   if (i > 1) {
      std::cout << "[ActiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Quat quat = this->getAttitude(this->wheelRoot_);
   osg::Vec3 axis(0, 1, 0);

   return quat * axis;

}

void ActiveWheelModel::getMotors(
      std::vector<boost::shared_ptr<Motor> >& motors) {
   motors.resize(1);
   motors[0] = this->motor_;
}

}
