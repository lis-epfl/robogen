/*
 * @(#) EvolverLog.h   1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
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

#ifndef EVOLVERLOG_H_
#define EVOLVERLOG_H_

#include <fstream>
#include "evolution/engine/Population.h"

namespace robogen {

class EvolverLog {
public:
	/**
	 * Initiate evolver log. Creates a directory according to the current time
	 * with BestAvgStd.txt inside and a copy of the generating configuration
	 * file.
	 * @param confFile evolution configuration file
	 */
	EvolverLog(std::string confFile);

	virtual ~EvolverLog();

	/**
	 * Logs an evaluated population, writing into BestAvgStd.txt and writing
	 * the best individual to file.
	 * @param step step iterator to be written to log file
	 * @param population Population to be checkpointed. Non-const on purpose,
	 * as population->best() calls population->sort()
	 */
	void logGeneration(int step, Population &population);

private:
	/**
	 * Log directory
	 */
	std::string logPath_;
	/**
	 * File stream to BestAvgStd.txt
	 */
	std::ofstream bestAvgStd_;
};

} /* namespace robogen */
#endif /* EVOLVERLOG_H_ */
