/*
 * @(#) NeuralNetworkRepresentation.cpp   1.0   Aug 28, 2013
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

#include "evolution/NeuralNetworkRepresentation.h"
#include <queue>
#include <sstream>
#include <stdexcept>

namespace robogen {

NeuralNetworkRepresentationException::NeuralNetworkRepresentationException(
		const std::string& w) : std::runtime_error(w){}

NeuralNetworkRepresentation::NeuralNetworkRepresentation(
		std::vector<std::string> motors, std::vector<std::string> sensors) {
	motors_.insert(motors.begin(), motors.end());
	sensors_.insert(sensors.begin(), sensors.end());
}

NeuralNetworkRepresentation::~NeuralNetworkRepresentation() {
}

void NeuralNetworkRepresentation::setWeight(std::string from, std::string to,
		double value){
	if (sensors_.find(from)==sensors_.end()&&motors_.find(from)==motors_.end()){
		std::stringstream ss;
		ss << "Specified weight input node " << from << " is neither in the sensor "\
				"nor the motor cache of the neural network. Candidates are:\n";
		for (std::set<std::string>::iterator it = sensors_.begin();
				it != sensors_.end(); ++it){
			ss << *it << ", ";
		}
		for (std::set<std::string>::iterator it = motors_.begin();
				it != motors_.end(); ++it){
			ss << *it << ", ";
		}
		throw NeuralNetworkRepresentationException(ss.str());
	}
	if (motors_.find(to)==motors_.end()){
		std::stringstream ss;
		ss << "Specified weight output node " << to << " is not in the "\
				"motor cache of the neural network. Candidates are:\n";
		for (std::set<std::string>::iterator it = motors_.begin();
				it != motors_.end(); ++it){
			ss << *it << ", ";
		}
		throw NeuralNetworkRepresentationException(ss.str());
	}
	weights_[std::pair<std::string, std::string>(from, to)] = value;
}

void NeuralNetworkRepresentation::setBias(std::string motor, double value){
	if (motors_.find(motor)==motors_.end()){
		std::stringstream ss;
		ss << "Specified bias node " << motor << " is not in the "\
				"motor cache of the neural network. Candidates are:\n";
		for (std::set<std::string>::iterator it = motors_.begin();
				it != motors_.end(); ++it){
			ss << *it << ", ";
		}
		throw NeuralNetworkRepresentationException(ss.str());
	}
	bias_[motor] = value;
}

} /* namespace robogen */
