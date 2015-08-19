/*
 * @(#) Simulator.h   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2014 Joshua Auerbach
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


#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>

#include "Robogen.h"
#include "config/RobogenConfig.h"
#include "scenario/Scenario.h"
#include "viewer/FileViewerLog.h"
#include "viewer/Viewer.h"

#define MIN_FITNESS (-10000.0)

namespace robogen{

/**
 * \brief Code for running a simulation
 *
 * Simulator wraps the ODE code, and should be used by other classes
 * that wish to run a simulation.
 *
 * Previously we had separate executables for Server, ServerViewer, and
 * FileViewer.  This involved a lot of duplicate code, so this class aims to
 * encapsulate all ODE code in one place
 */

enum result{
		SIMULATION_SUCCESS,
		SIMULATION_FAILURE,
		ACCELERATION_CAP_EXCEEDED
	};

/**
 * Runs the simulations
 * Relies on two extern variables. See .cpp.
 */
unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Robot &robotMessage, Viewer *viewer,
		boost::random::mt19937 &rng);


unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Robot &robotMessage, Viewer *viewer,
		boost::random::mt19937 &rng,
		bool onlyOnce, boost::shared_ptr<FileViewerLog> log);



}

#endif /* SIMULATOR_H_ */
