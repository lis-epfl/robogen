/*
 * @(#) NeuralNetwork.c   1.0   March 5, 2013
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
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "brain/NeuralNetwork.h"

#define PI 3.14159265358979323846
#define MAX_POS_SERVO 45*PI/180

void initNetwork(NeuralNetwork* network, unsigned int nInputs,
		unsigned int nOutputs, unsigned int nHidden,
		const float *weights, const float* params,
		const unsigned int *types) {

	unsigned int i = 0;

	/* Copy weights, params and types */
	memcpy(network->weight, weights,
			sizeof(float) * ((nInputs + nOutputs + nHidden) *
					(nOutputs + nHidden)));
	memcpy(network->params, params,
			sizeof(float) * (nOutputs + nHidden) * MAX_PARAMS);

	memcpy(network->types, types,
			sizeof(unsigned int) * (nOutputs + nHidden));


	network->nNonInputs = nOutputs + nHidden;

	/* Initialize states and data */
	for (i = 0; i < network->nNonInputs; ++i) {
		network->state[i] = 0.0;
	}
	
	for (i = 0; i < 3*network->nNonInputs; ++i){
		network->nData[i] = 0.0;
	}

	/* Initialize inputs */
	for (i = 0; i < nInputs; ++i) {
		network->input[i] = 0.0;
	}

	network->nInputs = nInputs;
	network->nOutputs = nOutputs;
	network->nHidden = nHidden;

}

void feed(NeuralNetwork* network, const float *input) {

	unsigned int i = 0;
	for (i = 0; i < network->nInputs; ++i) {
		network->input[i] = input[i];
	}
	
	// Check if normalized
	int count = 0;
	
	if(fabs(network->input[i]) > 1){
		printf("Input %d = %f\n", i, network->input[i]);
		
		if(network->input[i] > 1.)
			network->input[i] = 1.;
		else if(network->input[i] < -1.)
			network->input[i] = -1;
		
		printf("Input %d = %f\n", i, network->input[i]);
		
		count += 1;
	}
	
	if(count > 0)
		printf("Count of non normalized inputs : %d\n", count);
	

}

void step(NeuralNetwork* network, float time) {

	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int baseIndexOutputWeigths =
			network->nNonInputs * network->nInputs;
	float phi = network->nData[MAX_DATA*i + 1];	
			
	if (network->nOutputs == 0) {
		return;
	}

	/* For each hidden and output neuron, sum the state of all the incoming connection */

	for (i = 0; i < network->nNonInputs; ++i) { 

		network->activations[i] = 0;
		network->input_act[i] = 0;
		network->coupling[i] = 0;

		for (j = 0; j < network->nInputs; ++j) {
			network->activations[i] += network->weight[network->nNonInputs * j + i]
					* network->input[j];
					
			//printf("Weight In:%f\n",network->weight[network->nNonInputs * j + i]);
		}

		for (j = 0; j < network->nNonInputs; ++j) {
			network->activations[i] += network->weight[baseIndexOutputWeigths
					+ network->nNonInputs * j + i]
					* network->state[j];
// 			printf("Weight NonIn:%f\n",network->weight[baseIndexOutputWeigths
// 					+ network->nNonInputs * j + i]);

		}
		
		//New activation that takes only inputs informations
		for (j = 0; j < network->nInputs; ++j) {
			network->input_act[i] += network->weight[network->nNonInputs * j + i]
					* network->input[j];
					
			//printf("Input %d: %f\n",j, network->input[j]);
		}
	}
	
	/*For each CPG, sum the coupling term with all other CPGs*/
	for(i = 0; i < network->nOutputs; ++i){
		for (j = 0; j < network->nOutputs; ++j) {
			
			//No self connection term
			if(j != i)
				network->coupling[i] += network->weight[baseIndexOutputWeigths
					+ network->nNonInputs * j + i]*sin(phi - network->nData[MAX_DATA*j+2]); //- bias;
		}
	}

	/* Now add in biases and calculate new network state (or do appropriate operation for neuron type*/
	for (i = 0; i < network->nNonInputs; ++i) { 

		/* Save next state */
		if (network->types[i] == SIGMOID) {
			/* params are bias, gain */
			network->activations[i] -= network->params[MAX_PARAMS*i];
			network->state[i] = 1.0
				/ (1.0 + exp(-network->params[MAX_PARAMS*i+1] *
						network->activations[i]));
		} else if (network->types[i] == SIMPLE) {
			/* linear, params are bias, gain */

			network->activations[i] -= network->params[MAX_PARAMS*i];
			network->state[i] = network->params[MAX_PARAMS*i+1] *
					network->activations[i];

		} else if (network->types[i] == OSCILLATOR) {
			/* TODO should this consider inputs too?? */
			/* params are period, phase offset, gain (amplitude) */

			float period = network->params[MAX_PARAMS*i];
			float phaseOffset = network->params[MAX_PARAMS*i + 1];
			float gain = network->params[MAX_PARAMS*i + 2];
			network->state[i] = ((sin( (2.0*PI/period) *
				 (time - period * phaseOffset))) + 1.0) / 2.0;
            
            //printf("%f", network->activations[i]);
                 
			/* set output to be in [0.5 - gain/2, 0.5 + gain/2] */
			network->state[i] = (0.5 - (gain/2.0) +
					network->state[i] * gain);

            
            
		} else if (network->types[i] == CPG) {
			
			float d_phi = 0.;
            float omega = network->params[MAX_PARAMS*i];
            float phi = network->nData[MAX_DATA*i + 1];
            float prev_time = network->nData[MAX_DATA*i + 2];
            float dt = time - prev_time;
            
			
			d_phi = omega + network->activations[i];
			
            //printf("d_phi = %f \n", d_phi);
            //printf("phi = %f \n", phi);
            //printf("time = %f \n", time);
            
            
            phi = phi + (d_phi)*dt;
            
            network->state[i] = 0.5*(sin(2*PI*phi)+1);
            
            //printf("state = %f \n", network->state[i]);
            
            //phi = network->state[i];
            network->nData[MAX_DATA*i + 1] = phi;
            network->nData[MAX_DATA*i + 2] = time;
        }
	}

}

void fetch(const NeuralNetwork* network, float *output) {

	unsigned int i = 0;
	for (i = 0; i < network->nOutputs; ++i) {
		output[i] = network->state[i];
	}
}
