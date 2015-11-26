/*
 * @(#) PassiveWheelModel.h   1.0   Feb 13, 2013
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
#ifndef ROBOGEN_PASSIVE_WHEEL_MODEL_H_
#define ROBOGEN_PASSIVE_WHEEL_MODEL_H_

#include "model/Model.h"

namespace robogen {

/**
 * A wheel is modeled with 1 box, 1 cylinder and a hinge joint
 */
class PassiveWheelModel: public Model {

public:

   static const float MASS_SLOT;
   static const float MASS_WHEEL;
   static const float MASS_AXEL;

   static const float SLOT_WIDTH;
   static const float SLOT_THICKNESS;

   static const float WHEEL_AXEL_OFFSET;
   static const float WHEEL_THICKNESS;

   static const float AXEL_RADIUS;
   static const float AXEL_LENGTH;


   static const unsigned int B_SLOT_ID = 0;
   static const unsigned int B_WHEEL_ID = 1;
   static const unsigned int B_AXEL_ID = 2;

   PassiveWheelModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id,
		   float radius);

   virtual ~PassiveWheelModel();

   virtual bool initModel();

   virtual boost::shared_ptr<SimpleBody> getRoot();

   virtual boost::shared_ptr<SimpleBody> getSlot(unsigned int i);

   virtual osg::Vec3 getSlotPosition(unsigned int i);

   virtual osg::Vec3 getSlotOrientation(unsigned int i);

   virtual osg::Vec3 getSlotAxis(unsigned int i);

   float getRadius() const;

private:

   boost::shared_ptr<SimpleBody> wheelRoot_;

   float radius_;

};

}

#endif /* ROBOGEN_PASSIVE_WHEEL_MODEL_H_ */
