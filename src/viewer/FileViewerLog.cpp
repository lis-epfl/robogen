/*
 * FileViewerLog.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: Titus Cieslewski
 */
//TODO proper header comment

#include "viewer/FileViewerLog.h"
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/timer/timer.hpp>
#include <boost/shared_ptr.hpp>

#define LOG_DIRECTORY_PREFIX "FileViewer_"
#define LOG_DIRECTORY_FACET "%Y%m%d-%H%M%S"
#define TRAJECTORY_LOG_FILE "trajectoryLog.txt"
#define SENSOR_LABEL_FILE "sensorLabels.txt"
#define SENSOR_LOG_FILE "sensorLog.txt"
#define MOTOR_LOG_FILE "motorLog.txt"
#define TIME_LOG_FILE "timeLog.txt"
#define LOG_COL_WIDTH 12
#define OCTAVE_SCRIPT "robogenPlot.m"

namespace robogen{

FileViewerLog::FileViewerLog(std::string robotFile, std::string confFile,
		std::string obstacleFile, std::string startPosFile,
		std::vector<boost::shared_ptr<Sensor> > &sensors) {
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
		//TODO throw FileViewerLogError
		throw std::string("File viewer log can't create log directory.\n") +
				err.what();
	}

	// open trajectory log
	std::string trajectoryLogPath = logPathSs.str() + "/" + TRAJECTORY_LOG_FILE;
	trajectoryLog_.open(trajectoryLogPath.c_str());
	if (!trajectoryLog_.is_open()){
		throw std::string("Can't open trajectory log file");
	}
	// open sensor log
	std::string sensorLogPath = logPathSs.str() + "/" + SENSOR_LOG_FILE;
	sensorLog_.open(sensorLogPath.c_str());
	if (!sensorLog_.is_open()){
		throw std::string("Can't open sensor log file");
	}
	// open motor log
	std::string motorLogPath = logPathSs.str() + "/" + MOTOR_LOG_FILE;
	motorLog_.open(motorLogPath.c_str());
	if (!motorLog_.is_open()){
		throw std::string("Can't open motor log file");
	}

	// open time log
	std::string timeLogPath = logPathSs.str() + "/" + TIME_LOG_FILE;
	timeLog_.open(timeLogPath.c_str());
	if (!timeLog_.is_open()){
		throw std::string("Can't open time log file");
	}

	// copy robot file
	boost::filesystem::path robotFrom(robotFile);
	boost::filesystem::path robotTo(logPathSs.str()+"/"
			+robotFrom.filename().native());
	try{
		boost::filesystem::copy_file(robotFrom, robotTo);
	} catch (boost::filesystem::filesystem_error &err){
		throw std::string("Can't copy robot file\n")+err.what();
	}
	// copy configuration file
	boost::filesystem::path confFrom(confFile);
	boost::filesystem::path confTo(logPathSs.str()+"/"
			+confFrom.filename().native());
	try{
		boost::filesystem::copy_file(confFrom, confTo);
	} catch (boost::filesystem::filesystem_error &err){
		throw std::string("Can't copy configuration file\n")+err.what();
	}
	// copy obstacle file
	boost::filesystem::path obsFrom(obstacleFile);
	boost::filesystem::path obsTo(logPathSs.str()+"/"
			+obsFrom.filename().native());
	try{
		boost::filesystem::copy_file(obsFrom, obsTo);
	} catch (boost::filesystem::filesystem_error &err){
		throw std::string("Can't copy obstacle file\n")+err.what();
	}
	// copy starting position file
	boost::filesystem::path staPoFrom(startPosFile);
	boost::filesystem::path staPoTo(logPathSs.str()+"/"
			+staPoFrom.filename().native());
	try{
		boost::filesystem::copy_file(staPoFrom, staPoTo);
	} catch (boost::filesystem::filesystem_error &err){
		throw std::string("Can't copy starting position file\n")+err.what();
	}

	// copy octave script TODO copy any .m files maybe?
	boost::filesystem::path octFrom(OCTAVE_SCRIPT);
	boost::filesystem::path octTo(logPathSs.str()+"/"
			+octFrom.filename().native());
	try{
		boost::filesystem::copy_file(octFrom, octTo);
	} catch (boost::filesystem::filesystem_error &err){
		std::cout << "Didn't find " << OCTAVE_SCRIPT << "... " <<
				"If this script were in the execution directory, "\
				"I'd copy it to the result directory for you!" << std::endl;
	}

	// write out sensor Labels file
	// open sensor log
	std::string sensorLabelPath = logPathSs.str() + "/" + SENSOR_LABEL_FILE;
	std::ofstream sensorLabel;
	sensorLabel.open(sensorLabelPath.c_str());
	if (!sensorLabel.is_open()){
		throw std::string("Can't open sensor label file");
	}
	for (unsigned int i=0; i<sensors.size(); i++){
		sensorLabel << sensors[i]->getLabel() << std::endl;
	}
}

FileViewerLog::~FileViewerLog(){}

void FileViewerLog::logPosition(osg::Vec3 pos){
	trajectoryLog_ << std::setw(LOG_COL_WIDTH) << pos.x() << " " << pos.y() <<
			std::endl;
}

void FileViewerLog::logSensors(float sensorValues[], int n){
	for (int i=0; i<n; i++)
		sensorLog_ << std::setw(LOG_COL_WIDTH) << sensorValues[i] << " ";
	sensorLog_ << std::endl;
}

void FileViewerLog::logMotors(float motorValues[], int n){
	for (int i=0; i<n; i++)
		motorLog_ << std::setw(LOG_COL_WIDTH) << motorValues[i] << " ";
	motorLog_ << std::endl;
}

void FileViewerLog::logTimeInit(){
	timer_.reset(new boost::timer::auto_cpu_timer(timeLog_, "%w\n"));
}

void FileViewerLog::logTime(){
	timer_.reset();
}

}

