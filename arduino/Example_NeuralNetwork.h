/*
 * @(#) NeuralNetwork.h   1.0   March 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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

#define MAX_INPUT_NEURONS 13 
#define MAX_OUTPUT_NEURONS 6 

#define NB_ACC_GYRO_SENSORS 6
#define NB_LIGHTSENSORS 4 
#define NB_TOUCH_SENSORS 1 
#define NB_SERVOS_MOTORS 2 
int input[NB_LIGHTSENSORS+NB_TOUCH_SENSORS*2+NB_ACC_GYRO_SENSORS]={0,2,2,2,2,2,2,0,0,1,1,0}; 
float EAweight[MAX_INPUT_NEURONS * MAX_OUTPUT_NEURONS + MAX_OUTPUT_NEURONS * MAX_OUTPUT_NEURONS]={0.079476111,0.019745048,0.031090071,0.0021274097,0.046433792,0.064923175,0.0025055874,0.059230391,0.0372517,0.043461587,0.070071809,0.0080790538,0.022626026,0.047131188,0.012722298,0.064792238,0.025572548,0.017886767,0.01320517,0.0038833374,0.081825055,0.0041154632,0.0017530484,0.096878387,0.096202806,0.095555127,0.017343553,0.097623438}; 
float EABiasWeight[MAX_OUTPUT_NEURONS]={0.011117704,0.011170845}; 
float EAGain[MAX_OUTPUT_NEURONS]={1,1}; 

/*
 * No namespace here on purpose ;-)
 */
typedef struct {

	/*
	 * Given m input neurons and n output neurons
	 * m <= MAX_INPUT_NEURONS
	 * n <= MAX_OUTPUT_NEURONS
	 *
	 * One weight for each input-output connection (w_ij, input neuron i, 0 <= i <= m, output neuron j, 0 <= j <= n )
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
	float weight[MAX_INPUT_NEURONS * MAX_OUTPUT_NEURONS
			+ MAX_OUTPUT_NEURONS * MAX_OUTPUT_NEURONS];

	/*
	 * One bias for each output neuron
	 */
	float bias[MAX_OUTPUT_NEURONS];

	/*
	 * One gain for each output neuron
	 */
	float gain[MAX_OUTPUT_NEURONS];

	/*
	 * One state for each output neuron
	 * The state has double the space to store also the next
	 * value.
	 */
	float state[MAX_OUTPUT_NEURONS*2];

	/**
	 * Indicates at which index of the state array the current state starts
	 * (alternatively curStateStart = 0 or curStateStart = n/2)
	 */
	int curStateStart;

	/**
	 * Onje input state for each input neuron
	 */
	float input[MAX_INPUT_NEURONS];

	/**
	 * The number of inputs
	 */
	unsigned int nInputs;

	/**
	 * The number of outputs
	 */
	unsigned int nOutputs;

} NeuralNetwork;

#endif /* ROBOGEN_NEURAL_NETWORK_H_ */