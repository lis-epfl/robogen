/*
 * @(#) ArduinoNNCompiler.cpp   1.0   Sep 9, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
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
#include "model/sensors/Sensor.h"
#include "model/sensors/LightSensor.h"
#include "model/sensors/TouchSensor.h"
#include "model/sensors/SimpleSensor.h"
#include "evolution/representation/NeuralNetworkRepresentation.h"

namespace robogen {

ArduinoNNCompiler::ArduinoNNCompiler() {
}

ArduinoNNCompiler::~ArduinoNNCompiler() {
}

void ArduinoNNCompiler::compile(Robot &robot,
		std::ofstream &file){

	file << arduino::headTemplate << std::endl << std::endl;

	int nLight = 0, nTouch = 0, nServo = 0;
	std::vector<int> input;
	std::vector<boost::shared_ptr<Sensor> > sensors = robot.getSensors();
	std::vector<boost::shared_ptr<Motor> > motors = robot.getMotors();

	for (unsigned int i=0; i<sensors.size(); i++){
		// light sensor
		if (boost::dynamic_pointer_cast<LightSensor>(
				sensors[i])){
			input.push_back(arduino::LIGHT_SENSOR);
			file << "// Branch " << sensors[i]->getLabel() << " to " <<
					arduino::lightOrder[nLight++] << std::endl;
		}
		// touch sensor
		if (boost::dynamic_pointer_cast<TouchSensor>(
				sensors[i])){
			input.push_back(arduino::TOUCH_SENSOR);
			file << "// Branch " << sensors[i]->getLabel() << " to " <<
					arduino::touchOrder[nTouch++] << std::endl;
		}
		// IMU sensor
		if (boost::dynamic_pointer_cast<SimpleSensor>(
				sensors[i])){
			input.push_back(arduino::IMU);
		}
	}

	for (unsigned int i=0; i<motors.size(); ++i){
		NeuralNetworkRepresentation::ioPair id = motors[i]->getId();
		file << "// Branch " << id.first << " " << id.second  << " to " <<
				arduino::servoOrder[nServo++] << std::endl;
	}
	file << std::endl;
	file << "#define NB_LIGHTSENSORS " << nLight << std::endl;
	file << "#define NB_TOUCH_SENSORS " << nTouch << std::endl;
	file << "#define NB_SERVOS_MOTORS " << nServo << std::endl;
	file << std::endl;
	file << "int input[] = {";
	for (unsigned int i=0; i<input.size(); ++i) file << (i?", ":"") << input[i];
	file << "};" << std::endl;
	// process weights
	std::vector<double> weights;
	boost::shared_ptr<NeuralNetwork> brain = robot.getBrain();
	weights.insert(weights.end(),brain->weight,brain->weight+
			(brain->nInputs+brain->nOutputs)*brain->nOutputs);
	file << "float EAWeight[] = {";
	for (unsigned int i=0; i<weights.size(); ++i) file<<(i?", ":"")<<weights[i];
	file << "};" << std::endl;
	// process biases
	std::vector<double> biases;
	biases.insert(biases.end(),brain->bias,brain->bias+brain->nOutputs);
	file << "float EABiasWeight[] = {";
	for (unsigned int i=0; i<biases.size(); ++i) file << (i?", ":"")<<biases[i];
	file << "};" << std::endl;
	// set all gains to 1. TODO evolve gains!
	file << "float EAGain[] = {";
	for (unsigned int i=0; i<biases.size(); ++i) file << (i?", ":"") << 1.;
	file << "};" << std::endl << std::endl;

	file << arduino::footTemplate;
}

} /* namespace robogen */
