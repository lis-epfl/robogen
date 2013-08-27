/*
 * FileViewerLog.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Titus Cieslewski (dev@titus-c.ch)
 */
//TODO proper heading comment

#ifndef FILEVIEWERLOG_H_
#define FILEVIEWERLOG_H_

#include <osg/Vec3>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/timer/timer.hpp>
#include <boost/shared_ptr.hpp>
#include "model/sensors/Sensor.h"

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
	/**
	 * Initializes the directory, copies the inputs and opens the log files for
	 * writing.
	 */
	FileViewerLog(std::string robotFile, std::string confFile,
			std::string obstacleFile, std::string startPosFile,
			std::vector<boost::shared_ptr<Sensor> > &sensors);

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

	/**
	 * Initializes a timer to measure code execution duration
	 */
	void logTimeInit();

	/**
	 * Writes timer value to file
	 */
	void logTime();

private:
	std::ofstream trajectoryLog_;
	std::ofstream sensorLog_;
	std::ofstream motorLog_;
	std::ofstream timeLog_;
	boost::shared_ptr<boost::timer::auto_cpu_timer> timer_;
};

}

#endif /* FILEVIEWERLOG_H_ */
