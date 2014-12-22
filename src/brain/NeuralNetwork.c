/*
 * @(#) NeuralNetwork.c   1.0   March 5, 2013
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
#include <math.h>
#include <string.h>

#include "brain/NeuralNetwork.h"

#define PI 3.14159265358979323846

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

	/* Initialize states */
	for (i = 0; i < network->nNonInputs * 2; ++i) {
		network->state[i] = 0.0;
	}

	/* Initialize inputs */
	for (i = 0; i < nInputs; ++i) {
		network->input[i] = 0.0;
	}

	network->nInputs = nInputs;
	network->nOutputs = nOutputs;
	network->nHidden = nHidden;

	network->curStateStart = 0;

}

void feed(NeuralNetwork* network, const float *input) {

	unsigned int i = 0;
	for (i = 0; i < network->nInputs; ++i) {
		network->input[i] = input[i];
	}

}

void updateNeurons(NeuralNetwork* network, float *kn) {
	unsigned int i,j;
	for (i=0;i<network->nNonInputs;i++) {
		kn[i] = -network->currY[i];
		for (j=0;j<network->nNonInputs;j++) {
			float bias = network->params[MAX_PARAMS*j];
			float gain = network->params[MAX_PARAMS*j + 2];
			kn[i] += network->weight[network->nNonInputs * network->nInputs
			     					 + network->nNonInputs * j + i] *
			     		1./( 1. + exp( -gain * (network->currY[j] - bias) ) );
		}

		for (j=0;j<network->nInputs;j++) {
			kn[i] += network->weight[network->nNonInputs * j + i] *
						network->input[j];
		}

		/* divide by tau*/
		kn[i] /= network->params[MAX_PARAMS*i + 1];
	}
}

void updateCTRNN(NeuralNetwork* network, float stepSize){

	unsigned int i;

	for (i=0;i<network->nNonInputs;i++) {
		network->currY[i] = network->state[i];
	}
	updateNeurons(network,network->kn1);

	for (i=0;i<network->nNonInputs;i++) {
		network->currY[i] = network->state[i] +
								( stepSize * network->kn1[i]/2.0 );
	}
	updateNeurons(network, network->kn2);

	for (i=0;i<network->nNonInputs;i++) {
		network->currY[i] = network->state[i] +
								( stepSize * network->kn2[i]/2.0 );
	}
	updateNeurons(network, network->kn3);

	for (i=0;i<network->nNonInputs;i++) {
		network->currY[i] = network->state[i] + (stepSize * network->kn3[i] );
	}
	updateNeurons(network, network->kn4);


	for (i=0;i<network->nNonInputs;i++) {
		network->state[i] += (stepSize/6.0)*(
								network->kn1[i] +
								2*network->kn2[i] +
								2*network->kn3[i] +
								network->kn4[i]
							 );
	}
}


void step(NeuralNetwork* network, float time) {

	unsigned int nextState;
	unsigned int i = 0;
	unsigned int j = 0;

	if (network->nOutputs == 0) {
		return;
	}

	/* For each output neuron, sum the state of all the incoming connection */
	nextState = (network->curStateStart + network->nNonInputs)
			% network->nNonInputs;

	for (i = 0; i < network->nNonInputs; ++i) {

		float curNeuronActivation = 0;

		for (j = 0; j < network->nInputs; ++j) {
			curNeuronActivation += network->weight[network->nNonInputs * j + i]
					* network->input[j];
		}

		for (j = 0; j < network->nNonInputs; ++j) {
			curNeuronActivation += network->weight[
				    network->nNonInputs * network->nInputs
					+ network->nNonInputs * j + i]
					* network->state[network->curStateStart + j];
		}

		/* Save next state */
		if (network->types[i] == SIGMOID) {
			/* params are bias, gain */
			curNeuronActivation -= network->params[MAX_PARAMS*i];
			network->state[nextState + i] = 1.0
				/ (1.0 + exp(-network->params[MAX_PARAMS*i+1] *
						curNeuronActivation));
		} else if (network->types[i] == SIMPLE) {
			/* linear, params are bias, gain */

			curNeuronActivation -= network->params[MAX_PARAMS*i];
			network->state[nextState + i] = network->params[MAX_PARAMS*i+1] *
					curNeuronActivation;

		} else if (network->types[i] == OSCILLATOR) {
			/* TODO should this consider inputs too?? */
			/* params are period, phase offset, gain (amplitude) */


			float period = network->params[MAX_PARAMS*i];
			float phaseOffset = network->params[MAX_PARAMS*i + 1];
			float gain = network->params[MAX_PARAMS*i + 2];
			network->state[nextState + i] = ((sin( (2.0*PI/period) *
				 (time - period * phaseOffset))) + 1.0) / 2.0;

			/* set output to be in [0.5 - gain/2, 0.5 + gain/2] */
			network->state[nextState + i] = (0.5 - (gain/2.0) +
					network->state[nextState + i] * gain);

		}
	}

	network->curStateStart = nextState;

}

void fetch(const NeuralNetwork* network, float *output) {
	unsigned int i = 0;
	for (i = 0; i < network->nOutputs; ++i) {
		if (network->types[0] == CTRNN_SIGMOID) {
			/* CTRNN stores state before logistic applied, and doesn't use
			 * startState
			 */
			float bias = network->params[MAX_PARAMS*i];
			float gain = network->params[MAX_PARAMS*i + 2];
			output[i] = 1./( 1. + exp( -gain * (network->state[i] - bias) ) );

		} else {
			output[i] = network->state[network->curStateStart + i];
		}
	}

}
