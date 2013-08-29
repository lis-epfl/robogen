/*
 * @(#) RobotRepresentation.cpp   1.0   Aug 28, 2013
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

#include "evolution/RobotRepresentation.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>
#include <boost/regex.hpp>
#include "evolution/PartRepresentation.h"

namespace robogen{

/**
 * Helper function for decoding a part line of a robot text file.
 * @return true if successful read
 * @todo handle poor formatting VS empty line
 */
bool robotTextFileReadPartLine(std::ifstream &file, int &indent, int &slot,
		char &type, std::string &id, int &orientation,
		std::vector<double> &params){
	// match (0 or more tabs)(digit) (type) (id) (orientation) (parameters)
	static const boost::regex rx(
			"^(\\t*)(\\d) ([A-Z]) ([^\\s]+) (\\d)([ \\d\\.]*)$");
	boost::cmatch match;

	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)){
		// match[0]:whole string, match[1]:tabs, match[2]:slot, match[3]:type,
		// match[4]:id, match[5]:orientation, match[6]:parameters
		indent = match[1].length();
		slot = std::atoi(match[2].first);
		type = match[3].first[0];
		id = std::string(match[4]);
		orientation = std::atoi(match[5].first);
		double param;
		std::stringstream ss(match[6]);
		params.clear();
		while (ss >> param){
			params.push_back(param);
		}
		return true;
	}
	else{
		return false;
	}
}

/**
 * Helper function for decoding a weight line of a robot text file.
 * @return true if successful read
 * @todo handle poor formatting VS empty line
 */
bool robotTextFileReadWeightLine(std::ifstream &file, std::string &from,
		std::string &to, double &value){
	static const boost::regex rx("^([^\\s]+) ([^\\s]+) (\\d*\\.?\\d*)$");
	boost::cmatch match;

	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)){
		// match[0]:whole string, match[1]:from, match[2]:to, match[3]:value
		from.assign(match[1]);
		to.assign(match[2]);
		value = std::atof(match[3].first);
		return true;
	}
	else{
		return false;
	}
}

/**
 * Helper function for decoding a weight line of a robot text file.
 * @return true if successful read
 * @todo handle poor formatting VS empty line
 */
bool robotTextFileReadBiasLine(std::ifstream &file, std::string &node,
		double &value){
	static const boost::regex rx("^([^\\s]+) (\\d*\\.?\\d*)$");
	boost::cmatch match;

	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)){
		// match[0]:whole string, match[1]:node, match[2]:value
		node.assign(match[1]);
		value = std::atof(match[2].first);
		return true;
	}
	else{
		return false;
	}
}

RobotRepresentationException::RobotRepresentationException(
		const std::string& w): std::runtime_error(w){}

RobotRepresentation::RobotRepresentation(std::string robotTextFile){
	// open file
	std::ifstream file;
	file.open(robotTextFile.c_str());
	if (!file.is_open()){
		std::stringstream ss;
		ss << "Could not open robot text file " << robotTextFile;
		throw RobotRepresentationException(ss.str());
	}

	// prepare processing
	boost::shared_ptr<PartRepresentation> current;
	std::stack<boost::shared_ptr<PartRepresentation> > parentStack;
	int slot, orientation, indent;
	char type;
	std::string line, id;
	std::vector<double> params;

	// process root node
	if (!robotTextFileReadPartLine(file, indent, slot, type, id, orientation,
			params) || indent){
		throw RobotRepresentationException("Robot text file contains no or"\
				" poorly formatted root node");
	}
	current = PartRepresentation::create(type,id,orientation,params);
	bodyTree_ = current;

	// process other body parts
	while(robotTextFileReadPartLine(file, indent, slot, type, id, orientation,
			params)){
		if (!indent){
			throw RobotRepresentationException("Attempt to create "\
					"multiple root nodes!");
		}
		// indentation: Adding children to current
		if (indent>(parentStack.size())){
			parentStack.push(current);
		}
		// indentation: done adding children to top of parent stack
		if (indent<(parentStack.size())){
			parentStack.pop();
		}
		current = PartRepresentation::create(type,id,orientation,params);
		if (parentStack.top()->getChild(slot)){
			std::stringstream ss;
			ss << "Attempt to overwrite child " <<
					parentStack.top()->getChild(slot)->getId() << " of " <<
					parentStack.top()->getId() << " with " <<
					current->getId();
			throw RobotRepresentationException(ss.str());
		}
		parentStack.top()->setChild(slot, current);
	}

	// process brain
	std::string from, to;
	double value;
	neuralNetwork_.reset(new NeuralNetworkRepresentation(this->getMotors(),
			this->getSensors()));
	// weights
	while (robotTextFileReadWeightLine(file, from, to, value)){
		neuralNetwork_->setWeight(from, to, value);
	}
	// biases
	while (robotTextFileReadBiasLine(file, to, value)){
		neuralNetwork_->setBias(to, value);
	}
	file.close();
}

boost::shared_ptr<PartRepresentation> RobotRepresentation::getBody(){
	return bodyTree_;
}

std::vector<std::string> RobotRepresentation::getMotors(){
	std::vector<std::string> motors,temp;
	std::queue<boost::shared_ptr<PartRepresentation> > todo;
	todo.push(bodyTree_);
	while (!todo.empty()){
		temp = todo.front()->getMotors();
		motors.insert(motors.end(), temp.begin(), temp.end());
		for (int i=0; i<todo.front()->getArity(); i++){
			if (todo.front()->getChild(i+1))
				todo.push(todo.front()->getChild(i+1));
		}
		todo.pop();
	}
	return motors;
}

std::vector<std::string> RobotRepresentation::getSensors(){
	std::vector<std::string> sensors,temp;
	std::queue<boost::shared_ptr<PartRepresentation> > todo;
	todo.push(bodyTree_);
	while (!todo.empty()){
		temp = todo.front()->getSensors();
		sensors.insert(sensors.end(), temp.begin(), temp.end());
		for (int i=0; i<todo.front()->getArity(); i++){
			if (todo.front()->getChild(i+1))
				todo.push(todo.front()->getChild(i+1));
		}
		todo.pop();
	}
	return sensors;
}

}
