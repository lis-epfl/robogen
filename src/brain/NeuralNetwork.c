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

void initNetwork(NeuralNetwork* network, unsigned int nInputs,
		unsigned int nOutputs, unsigned int nHidden,
		const float *weights, const float* params) {

	unsigned int i = 0;

	/* Copy weights, bias and gains */
	memcpy(network->weight, weights,
			sizeof(float) * ((nInputs + nOutputs + nHidden) *
					(nOutputs + nHidden)));
	memcpy(network->params, params,
			sizeof(float) * (nOutputs + nHidden) * MAX_PARAMS);

	/* Initialize states */
	for (i = 0; i < nOutputs * 2; ++i) {
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

void step(NeuralNetwork* network) {

	unsigned int nextState;
	unsigned int i = 0;
	unsigned int j = 0;

	if (network->nOutputs == 0) {
		return;
	}

	/* For each output neuron, sum the state of all the incoming connection */
	nextState = (network->curStateStart + network->nOutputs)
			% network->nOutputs;

	for (i = 0; i < network->nOutputs; ++i) {

		float curNeuronActivation = 0;
		unsigned int baseIndexOutputWeigths = -1;

		for (j = 0; j < network->nInputs; ++j) {
			curNeuronActivation += network->weight[network->nOutputs * j + i]
					* network->input[j];
		}
		baseIndexOutputWeigths = network->nOutputs * network->nInputs;
		for (j = 0; j < network->nOutputs; ++j) {
			curNeuronActivation += network->weight[baseIndexOutputWeigths
					+ network->nOutputs * j + i]
					* network->state[network->curStateStart + j];
		}

		/* Save next state */
		curNeuronActivation -= network->params[MAX_PARAMS*i];
		network->state[nextState + i] = 1.0
				/ (1.0 + exp(-network->params[MAX_PARAMS*i+1] * curNeuronActivation));
	}

	network->curStateStart = nextState;

}

void fetch(const NeuralNetwork* network, float *output) {

	unsigned int i = 0;
	for (i = 0; i < network->nOutputs; ++i) {
		output[i] = network->state[network->curStateStart + i];
	}

}
