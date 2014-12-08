/*
 * @(#) ArduinoNNCompiler.h   1.0   Sep 9, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * based on previous work by:
 * Gregoire Heitz (gregoire.heitz@epfl.ch)
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

#ifndef ARDUINONNCOMPILER_H_
#define ARDUINONNCOMPILER_H_

#include <fstream>
#include "Robot.h"
#include "config/RobogenConfig.h"

namespace robogen {

/**
 * Class to encapsulate a routine that can write to file the Neural Network
 * representation of a given robot in a way that can be used by the Robogen
 * Arduino program
 */
class ArduinoNNCompiler {
public:
	ArduinoNNCompiler();
	virtual ~ArduinoNNCompiler();

	/**
	 * Compiles the given Robot's Neural Network to the given file stream
	 */
	static void compile(Robot &robot, RobogenConfig &config,
			std::ofstream &file);

	/**
	 * Gets the header and footer for NeuralNetwork.h
	 */
	static std::pair<std::string, std::string> getHeaderAndFooter();
};

} /* namespace robogen */
#endif /* ARDUINONNCOMPILER_H_ */
