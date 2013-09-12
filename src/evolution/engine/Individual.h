/*
 * @(#) Individual.h   1.0   Sep 12, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014
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

#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

#include "utils/network/TcpSocket.h"

namespace robogen {


/**
 * An extension to the robot representation that allows evolutionary
 * interaction. // TODO inheritance should be other way around!!!
 */
class Individual {
public:
	Individual();
	virtual ~Individual();

	/**
	 * Replaces copy constructor for derived classes.
	 */
	virtual Individual clone();

	/**
	 * Evaluate individual using given socket and given configuration file.
	 */
	void evaluate(TcpSocket *socket, std::string &confFile);

	/**
	 * @return fitness of individual
	 */
	double getFitness() const;

	/**
	 * TODO making this more general would make sense... getting close to
	 * ParadisEO :)
	 */
	virtual void getBrainGenome(std::vector<double*>weights,
			std::vector<double*>biases);

	/**
	 * @return evaluated_
	 */
	bool isEvaluated();

	/**
	 * Makes robot be not evaluated again
	 */
	void setDirty();

private:
	double fitness_;
	bool evaluated_;
};

/**
 * Used for ordering individuals by fitness
 */
bool operator >(const Individual &a, const Individual &b){
	return a.getFitness() > b.getFitness();
}

} /* namespace robogen */
#endif /* INDIVIDUAL_H_ */
