/*
 * @(#) ActuatedComponent.h   1.0   Feb 20, 2013
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
#ifndef ROBOGEN_ACTUATED_COMPONENT_H_
#define ROBOGEN_ACTUATED_COMPONENT_H_

#include <boost/shared_ptr.hpp>
#include <vector>

#include "model/motors/Motor.h"
#include "model/Model.h"


namespace robogen {

class ActuatedComponent : public Model {

public:

	/**
	 * Constructor
	 * @see Model
	 */
	ActuatedComponent(dWorldID odeWorld, dSpaceID odeSpace, std::string id) :
		Model(odeWorld, odeSpace, id) {}

	/**
	 * Destructor
	 */
	virtual ~ActuatedComponent() {}

	/**
	 * @return the available motors
	 */
	virtual void getMotors(std::vector<boost::shared_ptr<Motor> >& motors) = 0;
};

}


#endif /* ROBOGEN_ACTUATED_COMPONENT_H_ */
