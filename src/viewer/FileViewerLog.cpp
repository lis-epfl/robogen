/*
 * @(#) FileViewerLog.cpp   1.0   Aug 20, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Titus Cieslweski, Joshua Auerbach
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


#include "viewer/FileViewerLog.h"
#include <iostream>
#include <fstream>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
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
#define WEBGL_FILE "webGL.json"

namespace robogen{

FileViewerLog::FileViewerLog(std::string robotFile,
		std::string confFile,
		std::string obstacleFile,
		std::string startPosFile,
		std::string lightSourceFile,
		std::string scenarioFile,
		std::string logFolder,
		bool overwrite,
		bool writeWebGL) :
			robotFile_(robotFile),
			confFile_(confFile),
			obstacleFile_(obstacleFile),
			startPosFile_(startPosFile),
			lightSourceFile_(lightSourceFile),
			scenarioFile_(scenarioFile),
			logFolder_(logFolder),
			overwrite_(overwrite),
			writeWebGL_(writeWebGL) {
}

bool FileViewerLog::init(boost::shared_ptr<Robot> robot,
		boost::shared_ptr<RobogenConfig> config) {
	std::string tempPath = logFolder_;
	int curIndex = 0;

	if (overwrite_) {
		boost::filesystem::remove_all(tempPath);
	} else {
		while (boost::filesystem::is_directory(tempPath)) {
			std::stringstream newPath;
			newPath << logFolder_ << "_" << ++curIndex;
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
	std::string trajectoryLogPath = logPath_ + "/" + TRAJECTORY_LOG_FILE;
	trajectoryLog_.open(trajectoryLogPath.c_str());
	if (!trajectoryLog_.is_open()){
		std::cout << "Can't open trajectory log file" << std::endl;
		return false;
	}
	// open sensor log
	std::string sensorLogPath = logPath_ + "/" + SENSOR_LOG_FILE;
	sensorLog_.open(sensorLogPath.c_str());
	if (!sensorLog_.is_open()){
		std::cout << "Can't open sensor log file" << std::endl;
		return false;
	}
	// open motor log
	std::string motorLogPath = logPath_ + "/" + MOTOR_LOG_FILE;
	motorLog_.open(motorLogPath.c_str());
	if (!motorLog_.is_open()){
		std::cout << "Can't open motor log file" << std::endl;
		return false;
	}

	// compile neural network representation for Arduino
	// open motor log
	std::string arduinoNNPath = logPath_ + "/" + ARDUINO_NN_FILE;
	std::ofstream arduinoNN;
	arduinoNN.open(arduinoNNPath.c_str());
	if (!motorLog_.is_open()){
		std::cout << "Can't open arduino neural net log file" << std::endl;
		return false;
	}
	ArduinoNNCompiler::compile(*robot.get(),*config.get(),arduinoNN);

	// compile neural network representation for Body

	/*std::string bodyPath = logPath_ + "/" + BODY_FILE;
	std::ofstream body;
	body.open(bodyPath.c_str());
	if (!motorLog_.is_open()){
		std::cout << "Can't open body log file" << std::endl;
		return false;
	}
	BodyCompiler::compile(*robot.get(),body);*/

	//TODO get rid of so much code duplication below

	// copy robot file
	boost::filesystem::path robotFrom(this->robotFile_);
	std::stringstream ss;
	ss << logPath_ << "/" << robotFrom.filename().string();
	boost::filesystem::path robotTo(ss.str());
	try{
		boost::filesystem::copy_file(robotFrom, robotTo);
	} catch (boost::filesystem::filesystem_error &err){
		std::cout << "Can't copy robot file" << std::endl << err.what() <<
				std::endl;
		return false;
	}
	// copy configuration file
	boost::filesystem::path confFrom(this->confFile_);
	ss.str(""); ss.clear();
	ss << logPath_ << "/" << confFrom.filename().string();
	boost::filesystem::path confTo(ss.str());
	try{
		boost::filesystem::copy_file(confFrom, confTo);
	} catch (boost::filesystem::filesystem_error &err){
		std::cout << "Can't copy configuration file" << std::endl << err.what()
				<< std::endl;
		return false;
	}

	// copy obstacle file
	if (this->obstacleFile_.length() > 0) {
		boost::filesystem::path obsFrom(this->obstacleFile_);
		ss.str(""); ss.clear();
		ss << logPath_ << "/" << obsFrom.filename().string();
		boost::filesystem::path obsTo(ss.str());
		try{
			boost::filesystem::copy_file(obsFrom, obsTo);
		} catch (boost::filesystem::filesystem_error &err){
			std::cout << "Can't copy obstacle file\n"<< std::endl << err.what() <<
					std::endl;
			return false;
		}
	}

	// copy starting position file
	if (this->startPosFile_.length() > 0) {
		boost::filesystem::path staPoFrom(this->startPosFile_);
		ss.str(""); ss.clear();
		ss << logPath_ << "/" << staPoFrom.filename().string();
		boost::filesystem::path staPoTo(ss.str());
		try{
			boost::filesystem::copy_file(staPoFrom, staPoTo);
		} catch (boost::filesystem::filesystem_error &err){
			std::cout << "Can't copy starting position file" << std::endl <<
					err.what() << std::endl;
			return false;
		}
	}

	// copy light source file
	if (this->lightSourceFile_.length() > 0) {
		boost::filesystem::path lightSourcesFrom(this->lightSourceFile_);
		ss.str(""); ss.clear();
		ss << logPath_ << "/" << lightSourcesFrom.filename().string();
		boost::filesystem::path lightSourcesTo(ss.str());
		try{
			boost::filesystem::copy_file(lightSourcesFrom, lightSourcesTo);
		} catch (boost::filesystem::filesystem_error &err){
			std::cout << "Can't copy light sources file" << std::endl <<
					err.what() << std::endl;
			return false;
		}
	}

	// copy scenario file for scripted scenarios
	if (this->scenarioFile_.length() > 0) {
		boost::filesystem::path scenarioFrom(this->scenarioFile_);
		ss.str(""); ss.clear();
		ss << logPath_ << "/" << scenarioFrom.filename().string();
		boost::filesystem::path scenarioTo(ss.str());
		try{
			boost::filesystem::copy_file(scenarioFrom, scenarioTo);
		} catch (boost::filesystem::filesystem_error &err){
			std::cout << "Can't copy scenario script file" << std::endl <<
					err.what() << std::endl;
			return false;
		}
	}


	// copy any m files
	for (boost::filesystem::directory_iterator
			it(boost::filesystem::current_path());
			it != boost::filesystem::directory_iterator(); ++it){
		if (boost::filesystem::extension(it->path()) == ".m"){
			ss.str(""); ss.clear();
			ss << logPath_ << "/" << it->path().filename().string();
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
	std::string sensorLabelPath = logPath_ + "/" + SENSOR_LABEL_FILE;
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

std::string FileViewerLog::getWebGLFileName() {
	return logPath_ + "/" + WEBGL_FILE;
}

}

