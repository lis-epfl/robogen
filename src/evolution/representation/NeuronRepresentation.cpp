/*
 * @(#) NeuronRepresentation.cpp   1.0   Aug 31, 2013
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

#include "evolution/representation/NeuronRepresentation.h"
#include <sstream>

namespace robogen {

NeuronRepresentation::NeuronRepresentation(ioPair identification,
		unsigned int layer, const SigmoidNeuronParams &sigmoidParams) :
	identification_(identification), layer_(layer), bias_(sigmoidParams.bias_){
		std::stringstream ss;
		ss << identification.first + "-" << identification.second;
		id_ = ss.str();
		type_ = SIGMOID;
		//unused params
		phaseOffset_ = 0;
		frequency_ = 0;
		tau_ = 0;
		gain_ = 1.;
}

NeuronRepresentation::NeuronRepresentation(ioPair identification,
		unsigned int layer, const CTRNNSigmoidNeuronParams &ctrnnParams) :
	identification_(identification), layer_(layer), bias_(ctrnnParams.bias_),
	tau_(ctrnnParams.tau_){
		std::stringstream ss;
		ss << identification.first + "-" << identification.second;
		id_ = ss.str();
		type_ = CTRNN_SIGMOID;
		//unused params
		phaseOffset_ = 0;
		frequency_ = 0;
		gain_ = 1.;
}

NeuronRepresentation::NeuronRepresentation(ioPair identification,
		unsigned int layer, const OscillatorNeuronParams &oscillatorParams) :
	identification_(identification), layer_(layer),
	frequency_(oscillatorParams.frequency_),
	phaseOffset_(oscillatorParams.phaseOffset_) {
		std::stringstream ss;
		ss << identification.first + "-" << identification.second;
		id_ = ss.str();
		type_ = OSCILLATOR;
		//unused params
		bias_ = 0;
		tau_ = 0;
		gain_ = 1.;
}

NeuronRepresentation::~NeuronRepresentation() {
}

std::string &NeuronRepresentation::getId(){
	return id_;
}

bool NeuronRepresentation::isInput(){
	return (layer_==INPUT);
}

unsigned int NeuronRepresentation::getLayer() {
	return layer_;
}

unsigned int NeuronRepresentation::getType() {
	return type_;
}


void NeuronRepresentation::setBias(double value){
	bias_ = value;
}

double *NeuronRepresentation::getBiasPointer(){
	return &bias_;
}

ioPair NeuronRepresentation::getIoPair(){
	return identification_;
}

robogenMessage::Neuron NeuronRepresentation::serialize(){
	robogenMessage::Neuron serialization;
	serialization.set_id(id_);
	if(layer_ == OUTPUT)
		serialization.set_layer("output");
	else if(layer_ == INPUT)
		serialization.set_layer("input");
	else if(layer_ == HIDDEN)
		serialization.set_layer("hidden");
	serialization.set_bodypartid(identification_.first);
	serialization.set_ioid(identification_.second);
	if(type_ == SIGMOID) {
		serialization.set_type("sigmoid");
		serialization.set_bias(bias_);
	}
	else if (type_ == CTRNN_SIGMOID) {
		serialization.set_type("ctrnn_sigmoid");
		serialization.set_bias(bias_);
		serialization.set_tau(tau_);
	} else if (type_ == OSCILLATOR) {
		serialization.set_type("oscillator");
		serialization.set_frequency(frequency_);
		serialization.set_phaseoffset(phaseOffset_);
	}
	serialization.set_gain(gain_);
	return serialization;
}

} /* namespace robogen */
