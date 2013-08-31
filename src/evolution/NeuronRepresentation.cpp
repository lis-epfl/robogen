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

#include "evolution/NeuronRepresentation.h"

namespace robogen {

NeuronRepresentation::NeuronRepresentation(std::string id, std::string layer,
		double bias, std::string bodyPart, int ioId) :
		id_(id), layer_(layer), bias_(bias), bodyPart_(bodyPart), ioId_(ioId){
}

NeuronRepresentation::~NeuronRepresentation() {
}

std::string &NeuronRepresentation::getId(){
	return id_;
}

bool NeuronRepresentation::isInput(){
	return layer_ == "input";
}

void NeuronRepresentation::setBias(double value){
	bias_ = value;
}

robogenMessage::Neuron NeuronRepresentation::serialize(){
	robogenMessage::Neuron serialization;
	serialization.set_id(id_);
	serialization.set_layer(layer_);
	serialization.set_biasweight(bias_);
	serialization.set_bodypartid(bodyPart_);
	serialization.set_ioid(ioId_);
	return serialization;
}

} /* namespace robogen */
