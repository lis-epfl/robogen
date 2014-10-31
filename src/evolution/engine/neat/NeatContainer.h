/*
 * @(#) NeatContainer.h   1.0   Oct 30, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
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

#ifndef NETCONTAINER_H_
#define NETCONTAINER_H_

#include "config/EvolverConfiguration.h"
#include "evolution/engine/Population.h"
#include "evolution/neat/Population.h"
#include "evolution/neat/Genome.h"

namespace robogen {

// TODO transform to shared pointer?

class NeatContainer {

public:

	NeatContainer(boost::shared_ptr<EvolverConfiguration> &evoConf,
			boost::shared_ptr<Population> &population, unsigned int seed,
			boost::random::mt19937 &rng);
	virtual ~NeatContainer();

	bool fillPopulationWeights(boost::shared_ptr<Population> &population);

	bool produceNextGeneration(boost::shared_ptr<Population> &population);

private:

	bool fillBrain(NEAT::Genome *genome,
			boost::shared_ptr<RobotRepresentation> &robotRepresentation);

	typedef std::map<unsigned int, NEAT::Genome*> NeatIdToGenomeMap;
	typedef std::map<unsigned int, boost::shared_ptr<RobotRepresentation> >
		NeatIdToRobotMap;
	boost::shared_ptr<NEAT::Population> neatPopulation_;
	NeatIdToGenomeMap neatIdToGenomeMap_;
	NeatIdToRobotMap neatIdToRobotMap_;
	std::vector< boost::shared_ptr<RobotRepresentation> > unMappedRobots_;
	boost::shared_ptr<EvolverConfiguration> evoConf_;
	boost::random::mt19937 rng_;

	void printCurrentIds();



};

} /* namespace robogen */
#endif /* NETCONTAINER_H_ */
