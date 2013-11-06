/*
 * BodyCompiler.h
 *
 * Created on: Nov 5, 2013
 * Author: Deniz Aydin (deniz.aydin@epfl.ch)
 *
 * Previous work by:
 * Titus Cieslewski (dev@titus-c.ch)
 *
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
#ifndef BODYCOMPILER_H_
#define BODYCOMPILER_H_

#include <fstream>
#include "Robot.h"

namespace robogen {

/**
 * Class to encapsulate a routine that can write to file the body
 * representation of a given robot in a way that can be used by the 3D printing suite
 *  */
class BodyCompiler {
public:
	BodyCompiler();
	virtual ~BodyCompiler();

	/**
	 * Compiles the given Robot's Neural Network to the given file stream
	 */
	static void compile(Robot &robot, std::ofstream &file);
};

} /* namespace robogen */


#endif /* BODYCOMPILER_H_ */
