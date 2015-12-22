/*
 * @(#) RobogenCollision.h   1.0   March 21, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_COLLISION_H_
#define ROBOGEN_COLLISION_H_

#include "Robogen.h"
#include <boost/shared_ptr.hpp>
#include "scenario/Scenario.h"
#include "model/Model.h"

#include <map>

namespace robogen {

//useful container in case we want to have access to other data
//within the callback
class CollisionData {
public :

	CollisionData(boost::shared_ptr<Scenario> scenario);
	inline ~CollisionData() {
		geomModelMap_.clear();
	}
	inline boost::shared_ptr<Scenario> getScenario() {
		return scenario_;
	}
	bool ignoreCollision(dGeomID o1, dGeomID o2);

	bool isPartOfBody(dGeomID o1);

	inline bool hasObstacleCollisions() {
		return hasObstacleCollisions_;
	}
	void testObstacleCollisons(dGeomID o1, dGeomID o2);

	//unsigned int numCulled ;

private :
	boost::shared_ptr<Scenario> scenario_;
	std::map<dGeomID, boost::shared_ptr<Model> > geomModelMap_;
	bool hasObstacleCollisions_;

};



/**
 * Handles collisions between two ODE geometries.
 * Relies on two extern variables. See .cpp.
 * data should be a pointer to a RobogenConfig
 */
void odeCollisionCallback(void *data, dGeomID o1, dGeomID o2);

}


#endif /* ROBOGEN_COLLISION_H_ */
