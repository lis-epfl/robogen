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




// Branch Wheel1 0 to D11
// Branch Wheel2 0 to D5
// Branch S1 to A0
// Branch S2 to A1
// Branch S3 to A2
// Branch S4 to A3

#define NB_LIGHTSENSORS 0
#define NB_TOUCH_SENSORS 0
#define NB_IR_SENSORS 4
#define NB_SERVO_MOTORS 0
#define NB_ROTATION_MOTORS 2
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

/* irIndices 
* NONE if not irSensor
* otherwise index of irSensor
*/
const int irIndices[] = { NONE, NONE, NONE, NONE, NONE, NONE, 0, 1, 2, 3 };

/* double dimension Tab
* outputTab[i][0] is the value of the output port
* outputTab[i][1] is the type of the output : 
	0 for position control, and
	1 for velocity control
*/
const int outputTab[][2] = { {D11, 1}, {D5, 1} };

#define NEUTRAL_PIN D6

#define NB_INPUTS 10
#define NB_OUTPUTS 2
#define NB_HIDDEN 0
PROGMEM const float EAWeight[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -3, -3, 3, -0.806727, -2.87428, -3, -2.53897, -1.84666, 3, 3, -1.24157, 3};
PROGMEM const float EAParams[] = {1.79026, 1, -5.17356e+20, 0.111878, 1, 0};
unsigned int EATypes[] = {1, 1};


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
