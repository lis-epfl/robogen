/*
 * @(#) FakeRobotRepresentation.h   1.0   Sep 8, 2013
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

#ifndef FAKEROBOTREPRESENTATION_H
#define FAKEROBOTREPRESENTATION_H

#include <string>
#include <cmath>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include "robogen.pb.h"
#include "utils/network/TcpSocket.h"

namespace robogen{

/**
 * The goal of this class is to provide some object that is simple to mutate and
 * evolve, according to Ilya's suggestion.
 */
class RobotRepresentation{
public:
	/**
	 * Fake constructor
	 */
	RobotRepresentation(std::string robotTextFile){
		array_.resize(2);
	}

	/**
	 * Empty serialization
	 */
	robogenMessage::Robot serialize() const{
		robogenMessage::Robot message;
		message.set_id(1);
		message.set_configuration("");
		message.mutable_body();
		message.mutable_brain();
		return message;
	}

	void evaluate(Socket *socket, std::string conf){

	}

	/**
	 * @return fitness of string
	 * @todo eliminate dead code
	 */
	double getFitness() const{
		double retval = 0.;
		for (unsigned int i=0; i<array_.size(); i++){
			retval -= array_[i]*array_[i];
		}
		return retval;
	}

	/**
	 * Initialization
	 */
	void randomizeBrain(boost::random::mt19937	&rng){
		boost::random::uniform_real_distribution<double> rand(-1.,1.);
		for (unsigned int i=0; i<array_.size(); i++){
			array_[i] = rand(rng);
		}
	}

	/**
	 * Fake genome getter
	 */
	void getBrainGenome(std::vector<double*> &weights,
			std::vector<double*> &biases){
		weights.resize(1); biases.resize(1);
		for (int i=0; i<1; i++){
			weights[i] = &array_[i];
			biases[i] = &array_[1 + i];
		}
	}

	bool isEvaluated(){
		return true;
	}

	void setDirty(){

	}

private:
	std::vector<double> array_;
};

/**
 * Operator > returns true if fitness of a exceeds fitness of b
 */
inline bool operator >(const RobotRepresentation &a, const RobotRepresentation &b){
	return a.getFitness()>b.getFitness();
}

}

#endif /* FAKEROBOTREPRESENTATION_H */
