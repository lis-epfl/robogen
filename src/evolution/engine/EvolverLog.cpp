/*
 * @(#) EvolverLog.cpp   1.0   Sep 2, 2013
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

#include "evolution/engine/EvolverLog.h"
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include "evolution/engine/Population.h"
#include "evolution/representation/RobotRepresentation.h"

namespace robogen {

#define LOG_DIRECTORY_PREFIX "BrainEvolution_"
#define LOG_DIRECTORY_FACET "%Y%m%d-%H%M%S"
#define BAS_LOG_FILE "BestAvgStd.txt"
#define LOG_COL_WIDTH 12

EvolverLog::EvolverLog() {
	// create log directory with time stamp
	std::stringstream logPathSs;
	logPathSs << LOG_DIRECTORY_PREFIX;
	boost::posix_time::time_facet *myFacet =
			new boost::posix_time::time_facet(LOG_DIRECTORY_FACET);
	logPathSs.imbue(std::locale(std::cout.getloc(), myFacet));
	logPathSs << boost::posix_time::second_clock::local_time();
	boost::filesystem::path logPath(logPathSs.str());
	try{
		boost::filesystem::create_directories(logPath);
	} catch(const boost::filesystem::filesystem_error &err){
		//TODO throw EvolverLogError
		throw std::string("Evolver log can't create log directory.\n") +
				err.what();
	}

	// open trajectory log
	std::string basLogPath = logPathSs.str() + "/" + BAS_LOG_FILE;
	bestAvgStd_.open(basLogPath.c_str());
	if (!bestAvgStd_.is_open()){
		//TODO throw EvolverLogError
		throw std::string("Can't open Best/Average/STD log file");
	}
}

EvolverLog::~EvolverLog() {
}

void EvolverLog::logGeneration(int step, const Population &population){
	double best,average,stdev;
	population.getStat(best,average,stdev);
	bestAvgStd_ << std::setw(LOG_COL_WIDTH) << step << " " << best << " " <<
			average << " "  << stdev << std::endl;
}

} /* namespace robogen */
