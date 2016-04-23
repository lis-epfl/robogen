/*
 * @(#) ActiveWhegModel.h   1.0   Feb 27, 2013
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
#ifndef ROBOGEN_ACTIVE_WHEG_MODEL_H_
#define ROBOGEN_ACTIVE_WHEG_MODEL_H_

#include "model/ActuatedComponent.h"

namespace robogen {

/**
 * An active wheg is modeled with 2 boxes for the servo, 3 boxes for the spokes
 */
class ActiveWhegModel: public ActuatedComponent {

public:

   static const float MASS_SLOT;
   static const float MASS_SERVO;
   static const float MASS_WHEG;

   static const float SLOT_WIDTH;
   static const float SLOT_THICKNESS;
   static const float SERVO_WIDTH;
   static const float SERVO_LENGTH;
   static const float SERVO_HEIGHT;

   static const float WHEG_BASE_RADIUS;
   static const float WHEG_THICKNESS;
   static const float WHEG_WIDTH;
   static const float WHEG_ATTACHMENT_THICKNESS;

   static const unsigned int SLOT_A = 0;

   static const unsigned int B_SLOT_ID = 0;
   static const unsigned int B_SERVO_ID = 1;
   static const unsigned int B_WHEG_BASE = 2;
   static const unsigned int B_WHEG_SPOKE_1 = 3;
   static const unsigned int B_WHEG_SPOKE_2 = 4;
   static const unsigned int B_WHEG_SPOKE_3 = 5;

   ActiveWhegModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id,
		   float radius);

   virtual ~ActiveWhegModel();

   virtual bool initModel();

   virtual boost::shared_ptr<SimpleBody> getRoot();

   virtual boost::shared_ptr<SimpleBody> getSlot(unsigned int i);

   virtual osg::Vec3 getSlotPosition(unsigned int i);

   virtual osg::Vec3 getSlotOrientation(unsigned int i);

   virtual osg::Vec3 getSlotAxis(unsigned int i);

   virtual void getMotors(std::vector<boost::shared_ptr<Motor> >& motors);

   float getRadius() const;

private:

   boost::shared_ptr<SimpleBody> whegRoot_;

   boost::shared_ptr<Motor> motor_;

   float radius_;
};

}

#endif /* ROBOGEN_ACTIVE_WHEG_MODEL_H_ */
