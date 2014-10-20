/*
 * @(#) NeuronRepresentation.h   1.0   Aug 31, 2013
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

#ifndef NEURONREPRESENTATION_H_
#define NEURONREPRESENTATION_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include "robogen.pb.h"

namespace robogen {

/**
 * Body part Id, IO Id
 */
typedef std::pair<std::string,int> ioPair;

struct SigmoidNeuronParams{
	inline SigmoidNeuronParams(double bias): bias_(bias) {}
	double bias_;
};

struct CTRNNSigmoidNeuronParams{
	inline CTRNNSigmoidNeuronParams(double bias, double tau):
			bias_(bias), tau_(tau) {}
	double bias_, tau_;
};

struct OscillatorNeuronParams{
	inline OscillatorNeuronParams(double frequency, double phaseOffset):
			frequency_(frequency), phaseOffset_(phaseOffset) {}
	double frequency_;
	double phaseOffset_;
};

// TODO implement SUPG

class NeuronRepresentation {
public:

	enum neuronLayer{
		INPUT,
		OUTPUT,
		HIDDEN
	};

	enum neuronType{
		SIGMOID,
		CTRNN_SIGMOID,
		OSCILLATOR,
		SUPG
	};



	/**
	 * Creates a new neuron representation. A neuron is associated with a layer,
	 * and can be associated with a body part and the id of the corresponding
	 * IO (multiple sensors/motors per body part possible). Currently, the
	 * neuron needs to be associated to the latter, but in the future, hidden
	 * layers could be implemented by omitting the specification of the latter.
	 */
	NeuronRepresentation(ioPair identification,	unsigned int layer,
			const SigmoidNeuronParams &sigmoidParams);

	NeuronRepresentation(ioPair identification,	unsigned int layer,
				const CTRNNSigmoidNeuronParams &ctrnnParams);

	NeuronRepresentation(ioPair identification,	unsigned int layer,
			const OscillatorNeuronParams &oscillatorParams);

	virtual ~NeuronRepresentation();

	/**
	 * @return id of the Neuron
	 */
	std::string &getId();

	/**
	 * @return true if the neuron is in the input layer
	 */
	bool isInput();

	/**
	 * @return layer code of neuron
	 */
	unsigned int getLayer();

	/**
	 * @return type code of neuron
	 */
	unsigned int getType();

	/**
	 * @param value the bias value to be set
	 */
	void setBias(double value);

	/**
	 * @return pointer to bias, handle for mutator
	 */
	double *getBiasPointer();

	ioPair getIoPair();

	/**
	 * Transfer to proto buffer message.
	 */
	robogenMessage::Neuron serialize();

private:
	/**
	 * Identification (body part id, IO ID) of neuron
	 */
	ioPair identification_;

	/**
	 * Identifier of the neuron.
	 */
	std::string id_;

	/**
	 * Type identification. See enum neuronLayer
	 */
	unsigned int type_;

	/**
	 * Layer identification. See enum neuronLayer
	 */
	unsigned int layer_;

	/**
	 * Bias of the given Neuron
	 */
	double bias_;

	double tau_;

	double phaseOffset_;

	double frequency_;

	double gain_;
};

} /* namespace robogen */
#endif /* NEURONREPRESENTATION_H_ */
