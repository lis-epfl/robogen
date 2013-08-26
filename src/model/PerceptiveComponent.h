/*
 * @(#) PerceptiveComponent.h   1.0   Feb 27, 2013
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
#ifndef ROBOGEN_PERCEPTIVE_COMPONENT_H_
#define ROBOGEN_PERCEPTIVE_COMPONENT_H_

#include <boost/shared_ptr.hpp>
#include <vector>

#include "model/sensors/Sensor.h"
#include "model/Model.h"
#include "scenario/Environment.h"


namespace robogen {

class PerceptiveComponent : public Model {

public:

	/**
	 * Constructor
	 * @see Model
	 */
	PerceptiveComponent(dWorldID odeWorld, dSpaceID odeSpace, std::string id) :
		Model(odeWorld, odeSpace, id) {}

	/**
	 * Destructor
	 */
	virtual ~PerceptiveComponent() {}

	/**
	 * @return the available sensors
	 */
	virtual void getSensors(std::vector<boost::shared_ptr<Sensor> >& sensors) = 0;

	/**
	 * Updates the internal values of the sensors
	 */
	virtual void updateSensors(boost::shared_ptr<Environment>& env) = 0;
};

}


#endif /* ROBOGEN_PERCEPTIVE_COMPONENT_H_ */
