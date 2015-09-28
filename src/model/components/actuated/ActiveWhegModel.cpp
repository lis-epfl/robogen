/*
 * @(#) ActiveWhegModel.cpp   1.0   Feb 27, 2013
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
#include <cmath>
#include "model/components/actuated/ActiveWhegModel.h"
#include "model/motors/ServoMotor.h"

namespace robogen {

const float ActiveWhegModel::MASS_SLOT = inGrams(4);
const float ActiveWhegModel::MASS_SERVO = inGrams(9);
const float ActiveWhegModel::MASS_WHEG = inGrams(3);

const float ActiveWhegModel::SLOT_WIDTH = inMm(34);
const float ActiveWhegModel::SLOT_THICKNESS = inMm(1.5);
const float ActiveWhegModel::WHEG_BASE_RADIUS = inMm(9);
const float ActiveWhegModel::WHEG_THICKNESS = inMm(4);
const float ActiveWhegModel::WHEG_WIDTH = inMm(4);

const float ActiveWhegModel::SERVO_Z_OFFSET = inMm(9); // zCenter shift respect to slot z-center
const float ActiveWhegModel::SERVO_WIDTH = inMm(14);

//take off wheg thickness, because in reality overlap
const float ActiveWhegModel::SERVO_LENGTH = inMm(36.8) -
		ActiveWhegModel::WHEG_THICKNESS;

const float ActiveWhegModel::SERVO_HEIGHT = inMm(14);


const float ActiveWhegModel::SEPARATION = inMm(1.0);
const float ActiveWhegModel::X_SERVO = -ActiveWhegModel::SLOT_THICKNESS +
		ActiveWhegModel::SEPARATION + ActiveWhegModel::SERVO_LENGTH / 2;

const float ActiveWhegModel::X_WHEG_BASE = ActiveWhegModel::X_SERVO +
		ActiveWhegModel::SERVO_LENGTH / 2 +
        ActiveWhegModel::WHEG_THICKNESS / 2;

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


   dReal zServo = 0;
   boost::shared_ptr<SimpleBody> servo = this->addBox(MASS_SERVO,
		   osg::Vec3(X_SERVO, 0, zServo),
		   SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT, B_SERVO_ID);


   //need to scale the base to account for the larger whegs as right
   //now the whole model scales
   dReal whegBaseRadius = WHEG_BASE_RADIUS * getRadius()/0.03;

   boost::shared_ptr<SimpleBody> whegBase = this->addCylinder(MASS_WHEG / 4,
		   osg::Vec3(X_WHEG_BASE, 0, zServo), 1, whegBaseRadius,
		   WHEG_THICKNESS, B_WHEG_BASE);

   boost::shared_ptr<SimpleBody> spoke1 = this->addBox(MASS_WHEG / 4,
         osg::Vec3(X_WHEG_BASE, 0, zServo + whegBaseRadius + getRadius() / 2),
         WHEG_THICKNESS, WHEG_WIDTH, getRadius(), B_WHEG_SPOKE_1);

   boost::shared_ptr<SimpleBody> spoke2 = this->addBox(MASS_WHEG / 4,
		   osg::Vec3(X_WHEG_BASE, 0, zServo),
		   WHEG_THICKNESS, WHEG_WIDTH, getRadius(), B_WHEG_SPOKE_2);

   boost::shared_ptr<SimpleBody> spoke3 = this->addBox(MASS_WHEG / 4,
		   osg::Vec3(X_WHEG_BASE, 0, zServo),
		   WHEG_THICKNESS, WHEG_WIDTH, getRadius(), B_WHEG_SPOKE_3);

   // Position spokes
   osg::Quat rotation;

   //TODO
#if 0

   float rotationSpoke1 = 60;
   float rotationSpoke2 = 180;
   float rotationSpoke3 = 300;

   rotation.makeRotate(osg::inDegrees(rotationSpoke1), osg::Vec3(1, 0, 0));
   quatOde[0] = rotation.w();
   quatOde[1] = rotation.x();
   quatOde[2] = rotation.y();
   quatOde[3] = rotation.z();
   dBodySetQuaternion(spoke1, quatOde);


   rotation.makeRotate(osg::inDegrees(rotationSpoke2), osg::Vec3(1, 0, 0));
   quatOde[0] = rotation.w();
   quatOde[1] = rotation.x();
   quatOde[2] = rotation.y();
   quatOde[3] = rotation.z();
   dBodySetQuaternion(spoke2, quatOde);

   rotation.makeRotate(osg::inDegrees(rotationSpoke3), osg::Vec3(1, 0, 0));
   quatOde[0] = rotation.w();
   quatOde[1] = rotation.x();
   quatOde[2] = rotation.y();
   quatOde[3] = rotation.z();
   dBodySetQuaternion(spoke3, quatOde);

   // Move center of spokes
   osg::Vec3 newPosSpoke1(X_WHEG_BASE, 0, zServo);
   newPosSpoke1 += osg::Vec3(0,
         (whegBaseRadius + getRadius() / 2)
               * std::cos(osg::inDegrees(90.0 + rotationSpoke1)),
         (whegBaseRadius + getRadius() / 2)
               * std::sin(osg::inDegrees(90.0 + rotationSpoke1)));
   dBodySetPosition(spoke1, newPosSpoke1.x(), newPosSpoke1.y(),
         newPosSpoke1.z());


   osg::Vec3 newPosSpoke2(X_WHEG_BASE, 0, zServo);
   newPosSpoke2 += osg::Vec3(0,
         (whegBaseRadius + getRadius() / 2)
               * std::cos(osg::inDegrees(90.0 + rotationSpoke2)),
         (whegBaseRadius + getRadius() / 2)
               * std::sin(osg::inDegrees(90.0 + rotationSpoke2)));
   dBodySetPosition(spoke2, newPosSpoke2.x(), newPosSpoke2.y(),
         newPosSpoke2.z());

   osg::Vec3 newPosSpoke3(X_WHEG_BASE, 0, zServo);
   newPosSpoke3 += osg::Vec3(0,
         (whegBaseRadius + getRadius() / 2)
               * std::cos(osg::inDegrees(90.0 + rotationSpoke3)),
         (whegBaseRadius + getRadius() / 2)
               * std::sin(osg::inDegrees(90.0 + rotationSpoke3)));
   dBodySetPosition(spoke3, newPosSpoke3.x(), newPosSpoke3.y(),
         newPosSpoke3.z());

   // Create joints to hold pieces in position

   // slot <slider> servo
   this->fixBodies(whegRoot_, servo, osg::Vec3(1, 0, 0));
   this->fixBodies(whegBase, spoke1, osg::Vec3(1, 0, 0));
   this->fixBodies(whegBase, spoke2, osg::Vec3(1, 0, 0));
   this->fixBodies(whegBase, spoke3, osg::Vec3(1, 0, 0));

   // servo <(hinge)> wheg base
   dJointID joint = dJointCreateHinge(this->getPhysicsWorld(), 0);
   dJointAttach(joint, servo, whegBase);
   dJointSetHingeAxis(joint, 1, 0, 0);
   dJointSetHingeAnchor(joint, X_WHEG_BASE, 0, 0);

   // Create servo
   this->motor_.reset(
         new ServoMotor(joint, ServoMotor::DEFAULT_MAX_FORCE_ROTATIONAL,
        		 ioPair(this->getId(),0)));
#endif
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
