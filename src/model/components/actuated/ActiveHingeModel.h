/*
 * @(#) ActiveHingeModel.h   1.0   Feb 12, 2013
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
#ifndef ROBOGEN_ACTIVE_HINGE_MODEL_H_
#define ROBOGEN_ACTIVE_HINGE_MODEL_H_

#include "model/ActuatedComponent.h"

namespace robogen {

/**
 * An active hinge is modelled with 4 boxes and a hinge joint
 */
class ActiveHingeModel: public ActuatedComponent {

public:

	static const float MASS_SLOT;
	static const float MASS_SERVO;
	static const float MASS_FRAME;
	static const float SLOT_WIDTH;
	static const float SLOT_THICKNESS;
	static const float FRAME_LENGTH;
	static const float FRAME_HEIGHT;
	static const float FRAME_WIDTH;
	static const float FRAME_ROTATION_OFFSET;

	static const float SERVO_LENGTH;
	static const float SERVO_HEIGHT;
	static const float SERVO_WIDTH;
	static const float SERVO_ROTATION_OFFSET;
	static const float SERVO_POSITION_OFFSET;

	static const unsigned int SLOT_A = 0;
	static const unsigned int SLOT_B = 1;

	static const unsigned int B_SLOT_A_ID = 0;
	static const unsigned int B_SLOT_B_ID = 1;
	static const unsigned int B_FRAME_ID = 2;
	static const unsigned int B_SERVO_ID = 3;

	ActiveHingeModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id);

	virtual ~ActiveHingeModel();

	virtual bool initModel();

	virtual boost::shared_ptr<SimpleBody> getRoot();

	virtual boost::shared_ptr<SimpleBody> getSlot(unsigned int i);

	virtual osg::Vec3 getSlotPosition(unsigned int i);

	virtual osg::Vec3 getSlotOrientation(unsigned int i);

	virtual osg::Vec3 getSlotAxis(unsigned int i);

	virtual void getMotors(std::vector<boost::shared_ptr<Motor> >& motors);

private:

	boost::shared_ptr<SimpleBody> hingeRoot_;
	boost::shared_ptr<SimpleBody> hingeTail_;
	boost::shared_ptr<Motor> motor_;

};

}

#endif /* ROBOGEN_ACTIVE_HINGE_MODEL_H_ */
