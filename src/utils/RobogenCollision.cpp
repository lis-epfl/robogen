/*
 * @(#) RobogenCollision.cpp   1.0   March 21, 2013
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
#include "model/sensors/TouchSensor.h"
#include "utils/RobogenCollision.h"

// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;

namespace robogen {

void odeCollisionCallback(void*, dGeomID o1, dGeomID o2) {

	const int MAX_CONTACTS = 32; // maximum number of contact points per body

	// exit without doing anything if the two bodies are connected by a joint
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) {
		return;
	}

	dContact contact[MAX_CONTACTS];
	for (int i = 0; i < MAX_CONTACTS; i++) {
		contact[i].surface.slip1 = 0.01;
		contact[i].surface.slip2 = 0.01;
		contact[i].surface.mode = dContactSoftERP |
					dContactSoftCFM |
					dContactApprox1 |
					dContactSlip1 | dContactSlip2;

		contact[i].surface.mu = dInfinity;
		contact[i].surface.soft_erp = 0.96;
		contact[i].surface.soft_cfm = 0.01;



	}

	int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
			sizeof(dContact));

	if (collisionCounts != 0) {

		for (int i = 0; i < collisionCounts; i++) {

			dJointID c = dJointCreateContact(odeWorld, odeContactGroup,
					contact + i);
			dJointAttach(c, b1, b2);

		}
	}
}

}
