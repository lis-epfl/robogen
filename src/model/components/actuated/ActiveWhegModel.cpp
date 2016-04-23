/*
 * @(#) ActiveWhegModel.cpp   1.0   Feb 27, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Andrea Maesani, Joshua Auerbach
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
#include "model/components/actuated/ActiveWhegModel.h"
#include "model/motors/RotationMotor.h"

namespace robogen {

const float ActiveWhegModel::MASS_SLOT = inGrams(2);
const float ActiveWhegModel::MASS_SERVO = inGrams(11);
const float ActiveWhegModel::MASS_WHEG = inGrams(3);

const float ActiveWhegModel::SLOT_WIDTH = inMm(34);
const float ActiveWhegModel::SLOT_THICKNESS = inMm(1.5);
const float ActiveWhegModel::WHEG_BASE_RADIUS = inMm(9);
const float ActiveWhegModel::WHEG_THICKNESS = inMm(4);
const float ActiveWhegModel::WHEG_WIDTH = inMm(4);


const float ActiveWhegModel::SERVO_WIDTH = inMm(14);
const float ActiveWhegModel::SERVO_LENGTH = inMm(36.8);
const float ActiveWhegModel::SERVO_HEIGHT = inMm(14);

// attachment part now goes on exterior since students found easier to
// attach that way
const float ActiveWhegModel::WHEG_ATTACHMENT_THICKNESS = inMm(6);


ActiveWhegModel::ActiveWhegModel(dWorldID odeWorld, dSpaceID odeSpace,
      std::string id, float radius) :
      ActuatedComponent(odeWorld, odeSpace, id), radius_(radius) {

}

ActiveWhegModel::~ActiveWhegModel() {

}

float ActiveWhegModel::getRadius() const {
   return radius_;
}

bool ActiveWhegModel::initModel() {

  	// Create the 6 components of the wheg
	// now created directly with calls to this->add___
   
   whegRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0), SLOT_THICKNESS,
         SLOT_WIDTH, SLOT_WIDTH, B_SLOT_ID);


   dReal xServo = SERVO_LENGTH / 2 + SLOT_THICKNESS / 2;

   boost::shared_ptr<SimpleBody> servo = this->addBox(MASS_SERVO,
		   osg::Vec3(xServo, 0, 0),
		   SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT, B_SERVO_ID);

   //need to scale the base to account for the larger whegs as right
   //now the whole model scales
   dReal whegBaseRadius = WHEG_BASE_RADIUS * getRadius()/0.03;

   // wheg should be placed so outside of attachment aligns with end of
   // motor

   dReal xWheg = xServo + SERVO_LENGTH/2 - WHEG_ATTACHMENT_THICKNESS -
   			WHEG_THICKNESS/2;


   boost::shared_ptr<SimpleBody> whegBase = this->addCylinder(MASS_WHEG / 4,
		   osg::Vec3(xWheg, 0, 0), 1, whegBaseRadius,
		   WHEG_THICKNESS, B_WHEG_BASE);

   boost::shared_ptr<SimpleBody> spoke1 = this->addBox(MASS_WHEG / 4,
         osg::Vec3(xWheg, 0, 0 + whegBaseRadius + getRadius() / 2),
         WHEG_THICKNESS, WHEG_WIDTH, getRadius(), B_WHEG_SPOKE_1);

   boost::shared_ptr<SimpleBody> spoke2 = this->addBox(MASS_WHEG / 4,
		   osg::Vec3(xWheg, 0, 0),
		   WHEG_THICKNESS, WHEG_WIDTH, getRadius(), B_WHEG_SPOKE_2);

   boost::shared_ptr<SimpleBody> spoke3 = this->addBox(MASS_WHEG / 4,
		   osg::Vec3(xWheg, 0, 0),
		   WHEG_THICKNESS, WHEG_WIDTH, getRadius(), B_WHEG_SPOKE_3);

   // Position spokes
   osg::Quat rotation1, rotation2, rotation3;

   float rotationSpoke1 = 60;
   float rotationSpoke2 = 180;
   float rotationSpoke3 = 300;

   rotation1.makeRotate(osg::inDegrees(rotationSpoke1), osg::Vec3(1, 0, 0));
   spoke1->setAttitude(rotation1);

   rotation2.makeRotate(osg::inDegrees(rotationSpoke2), osg::Vec3(1, 0, 0));
   spoke2->setAttitude(rotation2);

   rotation3.makeRotate(osg::inDegrees(rotationSpoke3), osg::Vec3(1, 0, 0));
   spoke3->setAttitude(rotation3);

   // Move center of spokes
   osg::Vec3 newPosSpoke1(xWheg, 0, 0);
   newPosSpoke1 += osg::Vec3(0,
         (whegBaseRadius + getRadius() / 2)
               * std::cos(osg::inDegrees(90.0 + rotationSpoke1)),
         (whegBaseRadius + getRadius() / 2)
               * std::sin(osg::inDegrees(90.0 + rotationSpoke1)));
   spoke1->setPosition(newPosSpoke1);

   osg::Vec3 newPosSpoke2(xWheg, 0, 0);
   newPosSpoke2 += osg::Vec3(0,
         (whegBaseRadius + getRadius() / 2)
               * std::cos(osg::inDegrees(90.0 + rotationSpoke2)),
         (whegBaseRadius + getRadius() / 2)
               * std::sin(osg::inDegrees(90.0 + rotationSpoke2)));
   spoke2->setPosition(newPosSpoke2);

   osg::Vec3 newPosSpoke3(xWheg, 0, 0);
   newPosSpoke3 += osg::Vec3(0,
         (whegBaseRadius + getRadius() / 2)
               * std::cos(osg::inDegrees(90.0 + rotationSpoke3)),
         (whegBaseRadius + getRadius() / 2)
               * std::sin(osg::inDegrees(90.0 + rotationSpoke3)));
   spoke3->setPosition(newPosSpoke3);

   // Create joints to hold pieces in position

   // slot <slider> servo
   this->fixBodies(whegRoot_, servo);
   this->fixBodies(whegBase, spoke1);
   this->fixBodies(whegBase, spoke2);
   this->fixBodies(whegBase, spoke3);

   // servo <(hinge)> wheg base
   boost::shared_ptr<Joint> joint = this->attachWithHinge(servo, whegBase,
		   osg::Vec3(1,0,0), osg::Vec3(xWheg, 0, 0));

   // Create servo
   this->motor_.reset(
   		 new RotationMotor(ioPair(this->getId(),0), joint,
   				 RotationMotor::DEFAULT_MAX_FORCE_ROTATIONAL));
   return true;

}

boost::shared_ptr<SimpleBody> ActiveWhegModel::getRoot() {
   return whegRoot_;
}

boost::shared_ptr<SimpleBody> ActiveWhegModel::getSlot(unsigned int i) {

   if (i > 1) {
      std::cout << "[ActiveWhegModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   return whegRoot_;
}

osg::Vec3 ActiveWhegModel::getSlotPosition(unsigned int i) {

   if (i > 1) {
      std::cout << "[ActiveWhegModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Vec3 curPos = this->whegRoot_->getPosition();
   osg::Vec3 slotAxis = this->getSlotAxis(i);
   return curPos + slotAxis * (SLOT_THICKNESS / 2);

}

osg::Vec3 ActiveWhegModel::getSlotAxis(unsigned int i) {

   if (i > 1) {
      std::cout << "[ActiveWhegModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Quat quat = this->whegRoot_->getAttitude();
   osg::Vec3 axis(-1, 0, 0);

   return quat * axis;

}

osg::Vec3 ActiveWhegModel::getSlotOrientation(unsigned int i) {

   if (i > 1) {
      std::cout << "[ActiveWhegModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Quat quat = this->whegRoot_->getAttitude();
   osg::Vec3 axis(0, 1, 0);

   return quat * axis;

}

void ActiveWhegModel::getMotors(
      std::vector<boost::shared_ptr<Motor> >& motors) {
   motors.resize(1);
   motors[0] = this->motor_;
}

}
