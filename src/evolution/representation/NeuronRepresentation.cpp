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
#include <iostream>
namespace robogen {

void NeuronRepresentation::init(ioPair identification, unsigned int layer,
		unsigned int type) {
	identification_ = identification;
	layer_ = layer;
	type_ = type;
	std::stringstream ss;
	ss << identification.first + "-" << identification.second;
	id_ = ss.str();

	//defaults
	bias_ = 0;
	phaseOffset_ = 0;
	period_ = 0;
	tau_ = 0;
	gain_ = 1.;

}

NeuronRepresentation::NeuronRepresentation(ioPair identification,
		unsigned int layer, unsigned int type) {
	this->init(identification, layer, type);
}

NeuronRepresentation::NeuronRepresentation(ioPair identification,
		unsigned int layer) {
	this->init(identification, layer, SIMPLE);
}

NeuronRepresentation::NeuronRepresentation(ioPair identification,
		unsigned int layer, unsigned int type,
		const std::vector<double> params) {
	this->init(identification, layer, type);
	this->setParams(type, params);
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

void NeuronRepresentation::setParams(const std::vector<double> params) {
	setParams(type_, params);
}

void NeuronRepresentation::setParams(unsigned int type,
		const std::vector<double> params) {
	if (type == SIGMOID || type == SIMPLE) {
		bias_ = params[0];
	} else if (type == CTRNN_SIGMOID) {
		bias_ = params[0];
		tau_ = params[1];
	} else if (type == OSCILLATOR) {
		period_ = params[0];
		phaseOffset_ = params[1];
		gain_ = params[2]; // gain is amplitude for oscillators
	} else {
		std::cout<<"ALERT: ****\n Invalid type " << type <<"\n****\n";
	}
	type_ = type;
}

void NeuronRepresentation::getParamsPointers(std::vector<double*> &params){
	params.clear();
	if (type_ == SIGMOID) {
		params.push_back(&bias_);
	} else if (type_ == CTRNN_SIGMOID) {
		params.push_back(&bias_);
		params.push_back(&tau_);
	} else if (type_ == OSCILLATOR) {
		params.push_back(&period_);
		params.push_back(&phaseOffset_);
		params.push_back(&gain_);
	}
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
	if (type_ == SIMPLE) {
		serialization.set_type("simple");
	}
	else if(type_ == SIGMOID) {
		serialization.set_type("sigmoid");
		serialization.set_bias(bias_);
	}
	else if (type_ == CTRNN_SIGMOID) {
		serialization.set_type("ctrnn_sigmoid");
		serialization.set_bias(bias_);
		serialization.set_tau(tau_);
	} else if (type_ == OSCILLATOR) {
		serialization.set_type("oscillator");
		serialization.set_period(period_);
		serialization.set_phaseoffset(phaseOffset_);
	}
	serialization.set_gain(gain_);
	return serialization;
}

} /* namespace robogen */
