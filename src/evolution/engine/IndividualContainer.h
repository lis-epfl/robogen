/*
 * @(#) IndividualContainer.h   1.0   Sep 12, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014
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

#ifndef INDIVIDUALCONTAINER_H_
#define INDIVIDUALCONTAINER_H_

#include <vector>
#include "config/RobogenConfig.h"
#include "evolution/representation/RobotRepresentation.h"

namespace robogen {

// TODO transform to shared pointer?

class IndividualContainer: public std::vector<boost::shared_ptr<RobotRepresentation> > {

public:

	IndividualContainer();
	virtual ~IndividualContainer();

	/**
	 * Evaluate the population using the given scenario config and the given
	 * sockets to transmit the robot to simulator instances in parallel. Then
	 * order the population by fitness.
	 * @param robotConfig the robot configuration
	 * @param sockets a vector of Socket pointers. On each should be a simulator
	 */
	void evaluate(boost::shared_ptr<RobogenConfig> robotConfig, std::vector<Socket*> &sockets);

	/**
	 * Sorts individuals from best to worst.
	 * @param forceSort, will re-sort even if sorted_ flag is true
	 */
	void sort(bool forceSort=false);

	/**
	 * Append the contents of another IndividualContainer to this one.
	 */
	IndividualContainer &operator += (const IndividualContainer &other);

	/**
	 * @return whether all Individuals are evaluated
	 */
	bool areEvaluated() const;


protected:

	bool evaluated_;

private:

	bool sorted_;

};

} /* namespace robogen */
#endif /* INDIVIDUALCONTAINER_H_ */
