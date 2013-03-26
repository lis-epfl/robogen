/*
 * @(#) ScenarioFactory.h   1.0   Mar 13, 2013
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
#ifndef ROBOGEN_SCENARIO_FACTORY_H_
#define ROBOGEN_SCENARIO_FACTORY_H_

#include <boost/shared_ptr.hpp>
#include "Robogen.h"

namespace robogen {

class Scenario;
class RobogenConfig;

/**
 * Instantiates the correct scenario
 */
class ScenarioFactory {

public:

	/**
	 * Destructor
	 */
	virtual ~ScenarioFactory();

	/**
	 * Create a scenario
	 * @param config
	 * @return the scenario
	 */
	static boost::shared_ptr<Scenario> createScenario(boost::shared_ptr<RobogenConfig> config);

private:

	/**
	 * Disable instantiation
	 */
	ScenarioFactory();

};

}


#endif /* ROBOGEN_SCENARIO_FACTORY_H_ */
