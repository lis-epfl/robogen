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



class NeuronRepresentation {
public:

	enum neuronLayer{
		INPUT,
		OUTPUT,
		HIDDEN
	};

	enum neuronType{
		SIMPLE, //corresponds to inputs
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
	NeuronRepresentation(ioPair identification,	unsigned int layer);

	NeuronRepresentation(ioPair identification,	unsigned int layer,
			unsigned int type); // useful for copying neurons

	NeuronRepresentation(ioPair identification,	unsigned int layer,
			unsigned int type, const std::vector<double> params);

	virtual ~NeuronRepresentation();

	/**
	 * @return id of the Neuron
	 */
	std::string &getId();

	/**
	 * @param params the params to be set
	 */
	void setParams(const std::vector<double> params);

	/**
	 * Change the type of the neuron at the same time
	 */
	void setParams(unsigned int type, const std::vector<double> params);


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
	 * @param params reference to a vector to be filled with params pointers
	 */
	void getParamsPointers(std::vector<double*> &params);

	ioPair getIoPair();

	/**
	 * Transfer to proto buffer message.
	 */
	robogenMessage::Neuron serialize();

private:
	/**
	 * Code common to all constructors
	 */
	void init(ioPair identification, unsigned int layer, unsigned int type);



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

	double period_;

	double gain_;
};

} /* namespace robogen */
#endif /* NEURONREPRESENTATION_H_ */
