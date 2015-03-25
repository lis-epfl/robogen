/*
 * @(#) FileViewerLog.h   1.0   Aug 20, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc (guillaume.leclerc@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Titus Cieslweski, Joshua Auerbach
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


#ifndef WEBGLLOGGER_H_
#define WEBGLLOGGER_H_

#include <boost/shared_ptr.hpp>
#include "Robot.h"
#include <string>
#include <fstream>
using namespace std;

namespace robogen {

	class WebGLLogger {

		public :
			void logRobot(boost::shared_ptr<Robot> robot, double time);

			static boost::shared_ptr<WebGLLogger> getInstance();
			static void setFileName(string);

		private :
			static string fileName;
			WebGLLogger();
			ofstream fileHandler;
			static boost::shared_ptr<WebGLLogger> _instance;
	};
}

#endif

