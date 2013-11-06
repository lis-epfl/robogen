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
		bool isOutput, double bias) :
		identification_(identification), isOutput_(isOutput), bias_(bias){
	std::stringstream ss;
	ss << identification.first + "-" << identification.second;
	id_ = ss.str();
}

NeuronRepresentation::~NeuronRepresentation() {
}

std::string &NeuronRepresentation::getId(){
	return id_;
}

bool NeuronRepresentation::isInput(){
	return !isOutput_;
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
	serialization.set_layer(isOutput_?("output"):("input"));
	serialization.set_biasweight(bias_);
	serialization.set_bodypartid(identification_.first);
	serialization.set_ioid(identification_.second);
	return serialization;
}

} /* namespace robogen */
