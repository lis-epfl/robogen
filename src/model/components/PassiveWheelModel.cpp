/*
 * @(#) PassiveWheelModel.cpp   1.0   Feb 13, 2013
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
#include "model/components/PassiveWheelModel.h"

namespace robogen {

const float PassiveWheelModel::MASS_SLOT = inGrams(2);
const float PassiveWheelModel::MASS_AXEL = inGrams(2); //TODO verify
const float PassiveWheelModel::MASS_WHEEL = inGrams(4);

const float PassiveWheelModel::SLOT_WIDTH = inMm(34);
const float PassiveWheelModel::SLOT_THICKNESS = inMm(1.5);

const float PassiveWheelModel::WHEEL_THICKNESS = inMm(3);
const float PassiveWheelModel::WHEEL_AXEL_OFFSET = inMm(4.625);

const float PassiveWheelModel::AXEL_RADIUS = inMm(5);
const float PassiveWheelModel::AXEL_LENGTH = inMm(9.25);

PassiveWheelModel::PassiveWheelModel(dWorldID odeWorld, dSpaceID odeSpace,
      std::string id, float radius) :
      Model(odeWorld, odeSpace, id), radius_(radius) {

}

PassiveWheelModel::~PassiveWheelModel() {

}

float PassiveWheelModel::getRadius() const {
   return radius_;
}

bool PassiveWheelModel::initModel() {

	// Create the 2 components of the wheel,
	// now created directly with calls to this->add___


   wheelRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0),
         SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_ID);

   dReal xAxel =  SLOT_THICKNESS / 2 + AXEL_LENGTH / 2;

   boost::shared_ptr<SimpleBody> axel = this->addCylinder(MASS_AXEL,
   		   osg::Vec3(xAxel, 0, 0), 1,
   		   AXEL_RADIUS, AXEL_LENGTH, B_AXEL_ID);

   dReal xWheel = SLOT_THICKNESS/2  + WHEEL_AXEL_OFFSET;;

   boost::shared_ptr<SimpleBody> wheel = this->addCylinder(MASS_WHEEL,
		   osg::Vec3(xWheel, 0, 0), 1,
		   getRadius(), WHEEL_THICKNESS, B_WHEEL_ID);

   // Create joints to hold pieces in position

   this->fixBodies(wheelRoot_, axel);

   this->attachWithHinge(axel, wheel, osg::Vec3( 1, 0, 0),
		   osg::Vec3(xWheel, 0, 0) );

   return true;
}

boost::shared_ptr<SimpleBody> PassiveWheelModel::getRoot() {
   return wheelRoot_;
}

boost::shared_ptr<SimpleBody> PassiveWheelModel::getSlot(unsigned int i) {

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

   osg::Vec3 curPos = this->wheelRoot_->getPosition();
   osg::Vec3 slotAxis = this->getSlotAxis(i);
   return curPos + slotAxis * (SLOT_THICKNESS / 2);

}

osg::Vec3 PassiveWheelModel::getSlotAxis(unsigned int i) {

   if (i > 1) {
      std::cout << "[PassiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Quat quat = this->wheelRoot_->getAttitude();
   osg::Vec3 axis(-1, 0, 0);

   return quat * axis;

}

osg::Vec3 PassiveWheelModel::getSlotOrientation(unsigned int i) {

   if (i > 1) {
      std::cout << "[PassiveWheelModel] Invalid slot: " << i << std::endl;
      assert(i <= 1);
   }

   osg::Quat quat = this->wheelRoot_->getAttitude();
   osg::Vec3 axis(0, 1, 0);

   return quat * axis;

}

}
