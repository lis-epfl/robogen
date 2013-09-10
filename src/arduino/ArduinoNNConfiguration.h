/*
 * @(#) ArduinoNNConfiguration.h   1.0   Sep 10, 2013
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

#ifndef ARDUINONNCONFIGURATION_H_
#define ARDUINONNCONFIGURATION_H_

#include <string>

namespace robogen {

namespace arduino {

/**
 * Slots for light sensors
 */
std::string lightOrder[] = {"A0", "A1", "A2", "A3"};

/**
 * Slots for touch sensors
 */
std::string touchOrder[] = {"ROLL", "PITCH", "YAW", "AUX1", "D7", "D4"};

/**
 * Slots for servos
 */
std::string servoOrder[] = {"D9", "D10", "D5", "D6", "D11", "D13"};

/**
 * Integer codes for input type tab
 */
enum inputType{
	LIGHT_SENSOR,
	TOUCH_SENSOR,
	IMU
};

/**
 * String to be written at the head of the Arduino Neural Network file
 */
std::string headTemplate(""\
		"/*\n"\
		" * @(#) NeuralNetwork.h   1.0   March 5, 2013\n"\
		" *\n"\
		" * Andrea Maesani (andrea.maesani@epfl.ch)\n"\
		" *\n"\
		" * The ROBOGEN Framework\n"\
		" * Copyright © 2012-2013 Andrea Maesani\n"\
		" *\n"\
		" * Laboratory of Intelligent Systems, EPFL\n"\
		" *\n"\
		" * This file is part of the ROBOGEN Framework.\n"\
		" *\n"\
		" * The ROBOGEN Framework is free software: you can redistribute it and/or modify\n"\
		" * it under the terms of the GNU General Public License (GPL)\n"\
		" * as published by the Free Software Foundation, either version 3 of the License, or\n"\
		" * (at your option) any later version.\n"\
		" *\n"\
		" * This program is distributed in the hope that it will be useful,\n"\
		" * but WITHOUT ANY WARRANTY; without even the implied warranty of\n"\
		" * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"\
		" * GNU Lesser General Public License for more details.\n"\
		" *\n"\
		" * You should have received a copy of the GNU General Public License\n"\
		" * along with this program.  If not, see <http://www.gnu.org/licenses/>.\n"\
		" *\n"\
		" * @(#) $Id$\n"\
		" */\n"\
		" #ifndef ROBOGEN_NEURAL_NETWORK_H_\n"\
		" #define ROBOGEN_NEURAL_NETWORK_H_\n"\
		"\n"\
		"#define MAX_INPUT_NEURONS 13\n"\
		"#define MAX_OUTPUT_NEURONS 6\n"\
		"\n"\
		"#define NB_ACC_GYRO_SENSORS 6");

/**
 * String to be written at the end of the Arduino Neural Network file
 */
std::string footTemplate(""\
"\n"\
"/*\n"\
" * No namespace here on purpose ;-)\n"\
" */\n"\
"typedef struct {\n"\
"\n"\
"	/*\n"\
"	 * Given m input neurons and n output neurons\n"\
"	 * m <= MAX_INPUT_NEURONS\n"\
"	 * n <= MAX_OUTPUT_NEURONS\n"\
"	 *\n"\
"	 * One weight for each input-output connection (w_ij, input neuron i, 0 <= i <= m, output neuron j, 0 <= j <= n )\n"\
"	 * One weight for each output-output connection (wr_ij, output neuron i,j, 0 <= i,j <= n )\n"\
"	 *\n"\
"	 * The weights are saved as the concatenation by row of the following:\n"\
"	 * w_00 w_01 ... w_0n\n"\
"	 * w_10 w_11 ... w_1n\n"\
"	 * ...  ...  ... ...\n"\
"	 * w_m0 w_m1 ... w_mn\n"\
"	 *\n"\
"	 * wo_00 wo_01 ... wo_0n\n"\
"	 * wo_10 wo_11 ... wo_1n\n"\
"	 * ...  ...  ... ....\n"\
"	 * wo_n0 wo_n1 ... wo_nn\n"\
"	 */\n"\
"	float weight[MAX_INPUT_NEURONS * MAX_OUTPUT_NEURONS\n"\
"			+ MAX_OUTPUT_NEURONS * MAX_OUTPUT_NEURONS];\n"\
"\n"\
"	/*\n"\
"	 * One bias for each output neuron\n"\
"	 */\n"\
"	float bias[MAX_OUTPUT_NEURONS];\n"\
"\n"\
"	/*\n"\
"	 * One gain for each output neuron\n"\
"	 */\n"\
"	float gain[MAX_OUTPUT_NEURONS];\n"\
"\n"\
"	/*\n"\
"	 * One state for each output neuron\n"\
"	 * The state has double the space to store also the next\n"\
"	 * value.\n"\
"	 */\n"\
"	float state[MAX_OUTPUT_NEURONS*2];\n"\
"\n"\
"	/**\n"\
"	 * Indicates at which index of the state array the current state starts\n"\
"	 * (alternatively curStateStart = 0 or curStateStart = n/2)\n"\
"	 */\n"\
"	int curStateStart;\n"\
"\n"\
"	/**\n"\
"	 * Onje input state for each input neuron\n"\
"	 */\n"\
"	float input[MAX_INPUT_NEURONS];\n"\
"\n"\
"	/**\n"\
"	 * The number of inputs\n"\
"	 */\n"\
"	unsigned int nInputs;\n"\
"\n"\
"	/**\n"\
"	 * The number of outputs\n"\
"	 */\n"\
"	unsigned int nOutputs;\n"\
"\n"\
"} NeuralNetwork;\n"\
"\n"\
"#endif /* ROBOGEN_NEURAL_NETWORK_H_ */ ");

} /* namespace arduino */
} /* namespace robogen */
#endif /* ARDUINONNCONFIGURATION_H_ */
