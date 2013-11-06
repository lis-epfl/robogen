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
#include "arduino/ArduinoNNCompiler.h"
#include "printing/BodyCompiler.h"
#include "model/sensors/Sensor.h"

#define LOG_DIRECTORY_PREFIX "results/FileViewer_"
#define LOG_DIRECTORY_FACET "%Y%m%d-%H%M%S"
#define TRAJECTORY_LOG_FILE "trajectoryLog.txt"
#define SENSOR_LABEL_FILE "sensorLabels.txt"
#define SENSOR_LOG_FILE "sensorLog.txt"
#define MOTOR_LOG_FILE "motorLog.txt"
#define TIME_LOG_FILE "timeLog.txt"
#define ARDUINO_NN_FILE "NeuralNetwork.h"
#define BODY_FILE "bodyRepresentation.txt"
#define LOG_COL_WIDTH 12
#define OCTAVE_SCRIPT "robogenPlot.m"

namespace robogen{

FileViewerLog::FileViewerLog(){
}

bool FileViewerLog::init(std::string robotFile, std::string confFile,
		std::string obstacleFile, std::string startPosFile,
		boost::shared_ptr<Robot> robot) {
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
		std::cout << "File viewer log can't create log directory." << std::endl
				<< err.what() << std::endl;
		return false;
	}

	// open trajectory log
	std::string trajectoryLogPath = logPathSs.str() + "/" + TRAJECTORY_LOG_FILE;
	trajectoryLog_.open(trajectoryLogPath.c_str());
	if (!trajectoryLog_.is_open()){
		std::cout << "Can't open trajectory log file" << std::endl;
		return false;
	}
	// open sensor log
	std::string sensorLogPath = logPathSs.str() + "/" + SENSOR_LOG_FILE;
	sensorLog_.open(sensorLogPath.c_str());
	if (!sensorLog_.is_open()){
		std::cout << "Can't open sensor log file" << std::endl;
		return false;
	}
	// open motor log
	std::string motorLogPath = logPathSs.str() + "/" + MOTOR_LOG_FILE;
	motorLog_.open(motorLogPath.c_str());
	if (!motorLog_.is_open()){
		std::cout << "Can't open motor log file" << std::endl;
		return false;
	}

	// compile neural network representation for Arduino
	// open motor log
	std::string arduinoNNPath = logPathSs.str() + "/" + ARDUINO_NN_FILE;
	std::ofstream arduinoNN;
	arduinoNN.open(arduinoNNPath.c_str());
	if (!motorLog_.is_open()){
		std::cout << "Can't open arduino neural net log file" << std::endl;
		return false;
	}
	ArduinoNNCompiler::compile(*robot.get(),arduinoNN);

	// compile neural network representation for Body
	// open motor log
	std::string bodyPath = logPathSs.str() + "/" + BODY_FILE;
	std::ofstream body;
	body.open(bodyPath.c_str());
	if (!motorLog_.is_open()){
		std::cout << "Can't open body log file" << std::endl;
		return false;
	}
	BodyCompiler::compile(*robot.get(),body);

	// copy robot file
	boost::filesystem::path robotFrom(robotFile);
	std::stringstream ss;
	ss << logPathSs.str() << "/" << robotFrom.filename().string();
	boost::filesystem::path robotTo(ss.str());
	try{
		boost::filesystem::copy_file(robotFrom, robotTo);
	} catch (boost::filesystem::filesystem_error &err){
		std::cout << "Can't copy robot file" << std::endl << err.what() <<
				std::endl;
		return false;
	}
	// copy configuration file
	boost::filesystem::path confFrom(confFile);
	ss.str(""); ss.clear();
	ss << logPathSs.str() << "/" << confFrom.filename().string();
	boost::filesystem::path confTo(ss.str());
	try{
		boost::filesystem::copy_file(confFrom, confTo);
	} catch (boost::filesystem::filesystem_error &err){
		std::cout << "Can't copy configuration file" << std::endl << err.what()
				<< std::endl;
		return false;
	}
	// copy obstacle file
	boost::filesystem::path obsFrom(obstacleFile);
	ss.str(""); ss.clear();
	ss << logPathSs.str() << "/" << obsFrom.filename().string();
	boost::filesystem::path obsTo(ss.str());
	try{
		boost::filesystem::copy_file(obsFrom, obsTo);
	} catch (boost::filesystem::filesystem_error &err){
		std::cout << "Can't copy obstacle file\n"<< std::endl << err.what() <<
				std::endl;
		return false;
	}
	// copy starting position file
	boost::filesystem::path staPoFrom(startPosFile);
	ss.str(""); ss.clear();
	ss << logPathSs.str() << "/" << staPoFrom.filename().string();
	boost::filesystem::path staPoTo(ss.str());
	try{
		boost::filesystem::copy_file(staPoFrom, staPoTo);
	} catch (boost::filesystem::filesystem_error &err){
		std::cout << "Can't copy starting position file" << std::endl <<
				err.what() << std::endl;
		return false;
	}

	// copy any m files
	for (boost::filesystem::directory_iterator
			it(boost::filesystem::current_path());
			it != boost::filesystem::directory_iterator(); ++it){
		if (boost::filesystem::extension(it->path()) == ".m"){
			ss.str(""); ss.clear();
			ss << logPathSs.str() << "/" << it->path().filename().string();
			boost::filesystem::path mTo(ss.str());
			try{
				boost::filesystem::copy_file(it->path(), mTo);
			} catch (boost::filesystem::filesystem_error &err){
				std::cout << "Problem when copying m file " << it->path() <<
						std::endl;
			}
		}
	}

	// write out sensor Labels file
	// open sensor log
	std::string sensorLabelPath = logPathSs.str() + "/" + SENSOR_LABEL_FILE;
	std::ofstream sensorLabel;
	sensorLabel.open(sensorLabelPath.c_str());
	if (!sensorLabel.is_open()){
		std::cout << "Can't open sensor label file" << std::endl;
		return false;
	}
	std::vector<boost::shared_ptr<Sensor> > sensors = robot->getSensors();
	for (unsigned int i=0; i<sensors.size(); i++){
		sensorLabel << sensors[i]->getLabel() << std::endl;
	}
	return true;
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

}

