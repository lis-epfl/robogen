/*
 * @(#) NeuralNetwork.h   1.0   March 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_NEURAL_NETWORK_H_
#define ROBOGEN_NEURAL_NETWORK_H_

#define MAX_INPUT_NEURONS 20
#define MAX_OUTPUT_NEURONS 8

/*
 * set arbitrarily
 */
#define MAX_HIDDEN_NEURONS 20

/*
 * max is either (bias, tau, gain) or (phase offset, period, gain)
 */
#define MAX_PARAMS 3




// Branch Hip1 0 to D9
// Branch Hip2 0 to D10
// Branch Hip3 0 to D5
// Branch Hip4 0 to D6
// Branch Knee1 0 to D11
// Branch Knee2 0 to D13
// Branch Knee3 0 to ROLL
// Branch Knee4 0 to PITCH
// Branch IR1 0 to A0
// Branch IR2 0 to A1
// Branch IR3 0 to A2
// Branch IR4 0 to A3

#define NB_LIGHTSENSORS 0
#define NB_TOUCH_SENSORS 0
#define NB_IR_SENSORS 4
#define NB_SERVOS_MOTORS 8
#define NB_ACC_GYRO_SENSORS 6

#define ACTUATION_PERIOD 40

/* double dimension Tab
* inputTab[i][0] is the value of the input port
* inputTab[i][1] is the type of the input : 
	0 for lightSensor,
	1 for Touch sensor, and
	2 for Accelerometer and Gyroscope
	3 for IR sensor
*/
const int inputTab[][2] = { {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {A0, 3}, {A1, 3}, {A2, 3}, {A3, 3} };

/* double dimension Tab
* outputTab[i][0] is the value of the output port
* outputTab[i][1] is the type of the output : 
	0 for position control, and
	1 for velocity control
*/
const int outputTab[][2] = { {D9, 0}, {D10, 0}, {D5, 0}, {D6, 0}, {D11, 0}, {D13, 0}, {ROLL, 0}, {PITCH, 0} };

#define NB_INPUTS 10
#define NB_OUTPUTS 8
#define NB_HIDDEN 0
PROGMEM const float EAWeight[] = {-0.954982, 1.36992, -0.592611, 0.128227, -1.88935, -1.55253, -2.85677, 1.13961, -3, 3, -0.50755, 3, -2.86347, -3, 3, 2.80318, -0.775389, 0.935774, -2.01265, 2.39392, -1.59137, -2.12104, 2.97308, 1.73755, -3, 1.15937, -0.634366, -1.12455, -2.33158, -3, 2.60223, 0.854548, -1.0394, -0.3426, 2.62069, -1.87422, 2.36435, -0.580556, -0.842107, -0.864754, -0.275216, 3, 1.33945, -2.58632, -2.41779, -2.89692, -0.799209, 2.41427, 2.77562, -1.2976, 0.667586, -1.53167, 0.844275, 0.993757, -1.66375, -3, 3, 2.88426, -3, -1.1428, 2.97915, 1.806, 1.3751, 0.664077, 0.963294, -2.26186, -0.852061, 0.590064, -0.555294, 1.71827, 2.87933, -1.4741, 0.807379, -0.521197, 3, 2.31551, 0.491649, 2.89991, -1.89213, 2.34626, 2.30811, 0.791636, 2.19027, -2.41695, 3, 0.653835, -1.55969, -1.0428, -0.926017, -1.75582, -0.448575, 2.27968, 2.2635, -0.54371, 3, 1.50065, -0.63042, -0.591816, -0.414812, -2.25086, 1.96233, 2.44202, 0.922459, 2.23301, 1.92757, 0.664073, 0.65519, -2.29561, -2.10958, 3, -0.779189, -3, 1.5982, -3, -3, 0.493998, 0.0370664, 1.8948, 1.75534, -2.59373, 3, -3, -0.870935, -0.063758, 0.917309, -2.04566, 0.759281, -1.90194, -0.038426, -0.69894, 2.58125, 2.07083, -0.830881, 0.752178, 0.0988835, -0.257929, -2.9084, 0.152874, -3, 1.71289, 0.503856, -2.95145, 2.49927, 0.397824};
PROGMEM const float EAParams[] = {-2.89144, 1, 0, -1.42656, 1, 0, 1.4835, 1, 0, -2.34228, 1, 0, -1.80329, 1, 0, -3, 1, 0, -3, 1, 0, -2.81713, 1, 0};
unsigned int EATypes[] = {1, 1, 1, 1, 1, 1, 1, 1};


/*
 * No namespace here on purpose ;-)
 */

/*
 * Copied from NeuronRepresentation.h
 */
enum neuronType{
		SIMPLE, /* corresponds to inputs */
		SIGMOID,
		CTRNN_SIGMOID,
		OSCILLATOR,
		SUPG
};


typedef struct {

	/*
	 * Given m input neurons and n output neurons
	 * m <= MAX_INPUT_NEURONS
	 * n <= MAX_OUTPUT_NEURONS
	 *
	 * One weight for each input-output connection (w_ij, input neuron i, 0 <= i <= m, output neuron j, 0 <= j <= n
	 * One weight for each output-output connection (wr_ij, output neuron i,j, 0 <= i,j <= n )
	 *
	 * The weights are saved as the concatenation by row of the following:
	 * w_00 w_01 ... w_0n
	 * w_10 w_11 ... w_1n
	 * ...  ...  ... ...
	 * w_m0 w_m1 ... w_mn
	 *
	 * wo_00 wo_01 ... wo_0n
	 * wo_10 wo_11 ... wo_1n
	 * ...  ...  ... ....
	 * wo_n0 wo_n1 ... wo_nn
	 */
	#ifndef ARDUINO
	float weight[(MAX_INPUT_NEURONS + MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)
	             * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	#endif
	/*
	 * Params for hidden and output neurons, quantity depends on the type of
	 * neuron
	 */
	#ifndef ARDUINO
	float params[MAX_PARAMS * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	#endif

	/*
	 * One state for each output and hidden neuron
	 * activations will be used to temporarily store summed inputs before updating states
	 */
	float state[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	float activations[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];

	/**
	 * One input state for each input neuron
	 */
	float input[MAX_INPUT_NEURONS];


	/**
	 * Type of each non-input neuron
	 */
	#ifndef ARDUINO
	unsigned int types[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	#endif
	/**
	 * The number of inputs
	 */
	unsigned int nInputs;

	/**
	 * The number of outputs
	 */
	unsigned int nOutputs;

	/**
	 * The number of hidden units
	 */
	unsigned int nHidden;

	/**
	 * The number of non-inputs (i.e. nOutputs + nHidden)
	 */
	unsigned int nNonInputs;

} NeuralNetwork;

/**
 * TODO update this doc
 * Initializes a NeuralNetwork data structure
 * @param network the neural network
 * @param nInputs the number of inputs of the neural network
 * @param nOutputs the number of outputs of the neural network
 * @param weight weights of the neural network. Weights must be provided in the same order as
 *               specified in the NeuralNetwork structure
 * @param bias the bias of each output neuron
 * @param gain the gain of each output neuron
 */
void initNetwork(NeuralNetwork* network, unsigned int nInputs,
		unsigned int nOutputs, unsigned int nHidden,
		const float *weights, const float* params,
		const unsigned int *types);

/**
 * Feed the neural network with input values
 * @param network the neural network
 * @param input the input values, must be an array of m inputs
 */
void feed(NeuralNetwork* network, const float *input);

/**
 * Step the neural network of 1 timestep
 * @param network the neural network
 * @param time, amount of time elapsed since brain turned on
 * 				(needed for oscillators)
 */
void step(NeuralNetwork* network, float time);

/**
 * Read the output of the neural network
 * @param network the neural network
 * @param output the output of the neural network, must point to an area of memory of at least size n
 */
void fetch(const NeuralNetwork* network, float *output);


#endif /* ROBOGEN_NEURAL_NETWORK_H_ */
