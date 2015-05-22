/*
 * @(#) ArduinoNNCompiler.cpp   1.0   Sep 9, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * based on previous work by:
 * Gregoire Heitz (gregoire.heitz@epfl.ch)
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

#include "arduino/ArduinoNNCompiler.h"
#include "arduino/ArduinoNNConfiguration.h"
#include <iostream>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "model/ActuatedComponent.h"
#include "model/PerceptiveComponent.h"
#include "model/motors/Motor.h"
#include "model/motors/ServoMotor.h"
#include "model/sensors/Sensor.h"
#include "model/sensors/LightSensor.h"
#include "model/sensors/TouchSensor.h"
#include "model/sensors/SimpleSensor.h"
#include "evolution/representation/NeuralNetworkRepresentation.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

namespace robogen {

ArduinoNNCompiler::ArduinoNNCompiler() {
}

ArduinoNNCompiler::~ArduinoNNCompiler() {
}

void ArduinoNNCompiler::compile(Robot &robot, RobogenConfig &config,
		std::ofstream &file){

	std::pair<std::string, std::string> headerFooter = getHeaderAndFooter();

	file << headerFooter.first << std::endl << std::endl;

	int nLight = 0, nTouch = 0, nServo = 0;
	std::vector<int> input;
	std::vector<boost::shared_ptr<Sensor> > sensors = robot.getSensors();
	std::vector<boost::shared_ptr<Motor> > motors = robot.getMotors();


	// go through all motors first, so that they can use the "privileged"
	// digital pins (i.e. those which will not require any soldering)

	for (unsigned int i=0; i<motors.size(); ++i){
		ioPair id = motors[i]->getId();
		file << "// Branch " << id.first << " " << id.second  << " to " <<
				arduino::digitalOrder[nServo++] << std::endl;
	}

	for (unsigned int i=0; i<sensors.size(); i++){
		// light sensor
		if (boost::dynamic_pointer_cast<LightSensor>(
				sensors[i])){
			input.push_back(arduino::LIGHT_SENSOR);
			file << "// Branch " << sensors[i]->getLabel() << " to " <<
					arduino::analogOrder[nLight++] << std::endl;
		}
		// touch sensor
		if (boost::dynamic_pointer_cast<TouchSensor>(
				sensors[i])){
			input.push_back(arduino::TOUCH_SENSOR);
			file << "// Branch " << sensors[i]->getLabel() << " to " <<
					arduino::digitalOrder[nServo + (nTouch++)] << std::endl;
		}
		// IMU sensor
		if (boost::dynamic_pointer_cast<SimpleSensor>(
				sensors[i])){
			input.push_back(arduino::IMU);
		}
	}


	file << std::endl;
	file << "#define NB_LIGHTSENSORS " << nLight << std::endl;
	file << "#define NB_TOUCH_SENSORS " << nTouch << std::endl;
	file << "#define NB_SERVOS_MOTORS " << nServo << std::endl;
	file << "#define NB_ACC_GYRO_SENSORS 6" << std::endl;
	file << std::endl;

	float actuation_period_ms = config.getActuationPeriod() *
			config.getTimeStepLength() * 1000.0;

	file << "#define ACTUATION_PERIOD "  << ((int) actuation_period_ms)
			<< std::endl << std::endl;

	file << "int input[] = {";
	for (unsigned int i=0; i<input.size(); ++i) file << (i?", ":"") << input[i];
	file << "};" << std::endl;

	file << "int motor[] = {";
	for (unsigned int i = 0; i < motors.size(); ++i) {
		file << (i?", ":"");
		if (boost::dynamic_pointer_cast<ServoMotor>(motors[i])) {

			boost::shared_ptr<ServoMotor> motor =
					boost::dynamic_pointer_cast<ServoMotor>(motors[i]);
			file << (motor->isVelocityDriven() ?  arduino::VELOCITY_CONTROL :
							arduino::POSITION_CONTROL);
		} else {
			std::cout << "Error: unsupported motor!!";
			exit(EXIT_FAILURE);
		}
	}
	file << "};" << std::endl;

	boost::shared_ptr<NeuralNetwork> brain = robot.getBrain();
	file << "#define NB_INPUTS " << brain->nInputs << std::endl;
	file << "#define NB_OUTPUTS " << brain->nOutputs << std::endl;
	file << "#define NB_HIDDEN " << brain->nHidden << std::endl;



	// process weights
	std::vector<double> weights;

	weights.insert(weights.end(),brain->weight,brain->weight+
			(brain->nInputs + brain->nOutputs + brain->nHidden)*
			(brain->nOutputs + brain->nHidden));
	file << "PROGMEM const float EAWeight[] = {";
	for (unsigned int i=0; i<weights.size(); ++i) file<<(i?", ":"")<<weights[i];
	file << "};" << std::endl;

	// process params (biases, gains, etc)
	std::vector<double> params;
	params.insert(params.end(), brain->params, brain->params +
			(brain->nOutputs + brain->nHidden) * MAX_PARAMS);
	file << "PROGMEM const float EAParams[] = {";
	for (unsigned int i=0; i<params.size(); ++i) file << (i?", ":"")<<params[i];
	file << "};" << std::endl;

	// process types
	std::vector<unsigned int> types;
	types.insert(types.end(), brain->types, brain->types +
			(brain->nOutputs + brain->nHidden));
	file << "unsigned int EATypes[] = {";
	for (unsigned int i=0; i<types.size(); ++i) file << (i?", ":"")<<types[i];
	file << "};" << std::endl;


	file << headerFooter.second;
}

std::pair<std::string, std::string> ArduinoNNCompiler::getHeaderAndFooter() {
	#ifndef SOURCE_DIR
	    std::cerr << "SOURCE_DIR not properly specified from CMake."
	    		<< " Generating NeuralNetwork.h will not work."<< std::endl;
	#endif
	std::stringstream headerFileName;
	headerFileName << TOSTRING(SOURCE_DIR) << "/brain/NeuralNetwork.h";
	std::ifstream headerFile(headerFileName.str().c_str());
	std::string line;
	std::stringstream headerStream;
	std::stringstream footerStream;
	bool onFooter = false;
	while (std::getline(headerFile, line)) {
		if(line.find("HEADER_FOOTER_BREAK") != std::string::npos) {
			onFooter = true;
		} else if(onFooter) {
			footerStream << line << std::endl;
		} else {
			headerStream << line << std::endl;
		}

	}
	return std::make_pair(headerStream.str(), footerStream.str());
}


} /* namespace robogen */
