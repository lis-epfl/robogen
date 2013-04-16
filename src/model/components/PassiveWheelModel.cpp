/*
 * @(#) PassiveWheelModel.cpp   1.0   Feb 13, 2013
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
#include "model/components/PassiveWheelModel.h"

namespace robogen {

const float PassiveWheelModel::MASS_SLOT = inGrams(2.5);
const float PassiveWheelModel::MASS_WHEEL = inGrams(4);

const float PassiveWheelModel::SLOT_WIDTH = inMm(34);
const float PassiveWheelModel::SLOT_THICKNESS = inMm(10.75);
const float PassiveWheelModel::SLOT_WHEEL_OFFSET = inMm(7.5);
const float PassiveWheelModel::WHEEL_THICKNESS = inMm(3);

PassiveWheelModel::PassiveWheelModel(dWorldID odeWorld, dSpaceID odeSpace,
      float radius) :
      Model(odeWorld, odeSpace), radius_(radius) {

}

PassiveWheelModel::~PassiveWheelModel() {

}

float PassiveWheelModel::getRadius() const {
   return radius_;
}

bool PassiveWheelModel::initModel() {

   // Create the 4 components of the hinge
   wheelRoot_ = this->createBody(B_SLOT_ID);
   dBodyID wheel = this->createBody(B_WHEEL_ID);

   this->createBoxGeom(wheelRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
         SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

   dReal xWheel = SLOT_THICKNESS / 2 - (SLOT_THICKNESS - SLOT_WHEEL_OFFSET);

   this->createCylinderGeom(wheel, MASS_WHEEL, osg::Vec3(xWheel, 0, 0), 1,
         getRadius(), WHEEL_THICKNESS);

   // Create joints to hold pieces in position

   // slot <(hinge)> wheel
   dJointID joint = dJointCreateHinge(this->getPhysicsWorld(), 0);
   dJointAttach(joint, wheelRoot_, wheel);
   dJointSetHingeAxis(joint, 1, 0, 0);

   return true;

}

dBodyID PassiveWheelModel::getRoot() {
   return wheelRoot_;
}

dBodyID PassiveWheelModel::getSlot(unsigned int i) {

   if (i > 1) {
      std::cout << "[PassiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   return wheelRoot_;
}

osg::Vec3 PassiveWheelModel::getSlotPosition(unsigned int i) {

   if (i > 1) {
      std::cout << "[PassiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Vec3 curPos = this->getPosition(wheelRoot_);
   osg::Vec3 slotAxis = this->getSlotAxis(i);
   return curPos + slotAxis * (SLOT_THICKNESS / 2);

}

osg::Vec3 PassiveWheelModel::getSlotAxis(unsigned int i) {

   if (i > 1) {
      std::cout << "[PassiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Quat quat = this->getAttitude(this->wheelRoot_);
   osg::Vec3 axis(-1, 0, 0);

   return quat * axis;

}

osg::Vec3 PassiveWheelModel::getSlotOrientation(unsigned int i) {

   if (i > 1) {
      std::cout << "[PassiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Quat quat = this->getAttitude(this->wheelRoot_);
   osg::Vec3 axis(0, 1, 0);

   return quat * axis;

}

}
