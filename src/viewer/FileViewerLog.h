/*
 * @(#) FileViewerLog.h   1.0   Aug 20, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Titus Cieslweski, Joshua Auerbach
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

#ifndef FILEVIEWERLOG_H_
#define FILEVIEWERLOG_H_

#include <osg/Vec3>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/timer/timer.hpp>
#include <boost/shared_ptr.hpp>
#include "Robot.h"
#include "config/RobogenConfig.h"

namespace robogen{

/**
 * \brief Class for logging a robot's behavior
 *
 * FileViewerLog automatically creates a directory for each run of the
 * file viewer, using the time of the run for a unique name. The directory will
 * contain statistics that are useful to analyze the behavior of the robot,
 * i.e. a trajectory log and a log of the motor and sensor values.
 * Furthermore, the directory will contain all the inputs, so that the given
 * run can be repeated at any time.
 */
class FileViewerLog{
public:

	FileViewerLog(std::string robotFile,
		std::string confFile,
		std::string obstacleFile,
		std::string startPosFile,
		std::string lightSourceFile,
		std::string scenarioFile,
		std::string logFolder,
		bool overwrite = false,
		bool writeWebGL = false);

	/**
	 * Initializes the directory, copies the inputs and opens the log files for
	 * writing.
	 * @return true if successful
	 */
	bool init(boost::shared_ptr<Robot> robot,
			boost::shared_ptr<RobogenConfig> config);

	/**
	 * Closes all open file streams
	 */
	~FileViewerLog();

	/**
	 * Writes x and y of passed position to trajectory log file
	 */
	void logPosition(osg::Vec3);

	/**
	 * Writes sensor values from float array to sensor log file
	 */
	void logSensors(float sensorValues[], int n);

	/**
	 * Writes motor values from float array to motor log file
	 */
	void logMotors(float motorValues[], int n);

	inline bool isWriteWebGL() { return writeWebGL_; }

	std::string getWebGLFileName();

private:
	std::ofstream trajectoryLog_;
	std::ofstream sensorLog_;
	std::ofstream motorLog_;
	/**
	 * Log directory
	 */
	std::string logPath_;

	std::string robotFile_;
	std::string confFile_;
	std::string obstacleFile_;
	std::string startPosFile_;
	std::string lightSourceFile_;
	std::string scenarioFile_;
	std::string logFolder_;

	bool overwrite_;
	bool writeWebGL_;

};

}

#endif /* FILEVIEWERLOG_H_ */
