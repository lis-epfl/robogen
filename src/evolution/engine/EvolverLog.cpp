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
#include <iostream>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "evolution/representation/RobotRepresentation.h"
#include "evolution/engine/EvolverLog.h"
#include "evolution/engine/Population.h"
#include "utils/json2pb/json2pb.h"

namespace robogen {

#define BAS_LOG_FILE "BestAvgStd.txt"
#define GENERATION_BEST_PREFIX "GenerationBest-"

EvolverLog::EvolverLog(){
}

bool fixed_is_directory(std::string path) {
	boost::system::error_code errorCode;
	bool result = boost::filesystem::is_directory(path, errorCode);
	if (errorCode.value() == 2 ){
		return false;
	} else if (errorCode.value() != 0) {
		//this second call will fire the correct exception
		std::cout << errorCode.value() << std::endl << errorCode.message() << std::endl;
		return boost::filesystem::is_directory(path);
	} else {
		return result;
	}
}


bool EvolverLog::init(boost::shared_ptr<EvolverConfiguration> conf,
		boost::shared_ptr<RobogenConfig> robotConf,
		const std::string& logDirectory, bool overwrite, bool saveAll) {



	saveAll_ = saveAll;

	std::string tempPath = logDirectory;
	int curIndex = 0;
	if (overwrite) {
			boost::filesystem::remove_all(tempPath);
	} else {
		while (fixed_is_directory(tempPath)) {
			std::stringstream newPath;
			newPath << logDirectory << "_" << ++curIndex;
			tempPath = newPath.str();
		}
	}


	logPath_ = tempPath;
	boost::filesystem::path logPath(logPath_);

	try{
		boost::filesystem::create_directories(logPath);
	} catch(const boost::filesystem::filesystem_error &err){
		std::cout << err.what() << std::endl << "Evolver log can't create log"\
				" directory.\n" << std::endl;
		return false;
	}

	// open trajectory log
	std::string basLogPath = logPath_ + "/" + BAS_LOG_FILE;
	bestAvgStd_.open(basLogPath.c_str());
	if (!bestAvgStd_.is_open()){
		std::cout << "Can't open Best/Average/STD log file" << std::endl;
		return false;
	}

	// copy evolution configuration file
	copyConfFile(conf->confFileName);
	// copy simulator configuration file
	copyConfFile(conf->simulatorConfFile);
	// copy obstacle configuration file
	copyConfFile(robotConf->getObstacleFile());
	// copy light source configuration file
	copyConfFile(robotConf->getLightSourceFile());
	// copy start pos configuration file
	copyConfFile(robotConf->getStartPosFile());
	// copy robot file if doing just brain evolution
	if (conf->evolutionMode == EvolverConfiguration::BRAIN_EVOLVER) {
		copyConfFile(conf->referenceRobotFile);
	}
	// copy scenario file if using scripted scenario
	copyConfFile(robotConf->getScenarioFile());




	return true;
}

EvolverLog::~EvolverLog() {
}

void saveRobotJson(boost::shared_ptr<RobotRepresentation> robot,
		std::string fileName) {
	std::ofstream curRobotFile(fileName.c_str(),std::ios::out|std::ios::trunc);
	curRobotFile << pb2json(robot->serialize());
	curRobotFile.close();
}

bool EvolverLog::logGeneration(int generation, Population &population) {

	if (!population.areEvaluated()){
		std::cout << "EvolverLog::logGeneration(): Trying to log non-evaluated"\
				" population!" << std::endl;
		return false;
	}

	// log best, avg, stddev
	double best,average,stdev;
	population.getStat(best,average,stdev);
	std::cout << "Generation " << generation <<
				 ", Best: " << best << " Average: " << average << " STD: " <<
				 	 stdev << std::endl;
	bestAvgStd_ << generation << " " << best << " " <<
			average << " "  << stdev << std::endl;

	std::cout << "All fitnesses:";
	for(unsigned int i = 0; i<population.size(); ++i) {
		std::cout << " " << population[i]->getFitness();
	}
	std::cout << std::endl;

	// save robot file of best robot (don't do with fake robot representation)
	#ifndef FAKEROBOTREPRESENTATION_H

	boost::shared_ptr<RobotRepresentation> bestRobot = population.best();

	{
		std::stringstream ss;
		ss << logPath_ + "/" + GENERATION_BEST_PREFIX << generation << ".json";

		// De-comment to save protobuf binary file
		// ss << logPath_ + "/" + GENERATION_BEST_PREFIX << generation << ".dat
		// bestRobot->serialize().SerializeToOstream(&curRobotFile);
		saveRobotJson(bestRobot, ss.str());
	}


	if(saveAll_) {
		for(unsigned int i = 0; i<population.size(); ++i) {
			std::stringstream ss;
			ss << logPath_ + "/Generation-" << generation << "-Guy-" << (i+1)
					<< ".json";
			saveRobotJson(population[i], ss.str());
		}
	}


	#endif /* FAKEROBOTREPRESENTATION_H */

	return true;
}

void EvolverLog::copyConfFile(std::string fileName) {
	if (fileName.length() == 0)
		return;

	boost::filesystem::path confFrom(fileName);
	std::stringstream ss;
	ss << logPath_ << "/" << confFrom.filename().string();
	boost::filesystem::path confTo(ss.str());
	boost::filesystem::copy_file(confFrom, confTo);

}

} /* namespace robogen */
