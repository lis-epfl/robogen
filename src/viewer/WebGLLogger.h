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
#include <jansson.h>

namespace robogen {

	struct BodyDescriptor {
		boost::shared_ptr<Model> model;
		int bodyId;
	};

	class WebGLLogger {

		public :
			WebGLLogger(std::string in_filename, boost::shared_ptr<Robot> in_robot);
			void log(double dt);
			~ WebGLLogger();
			static const char* STRUCTURE_TAG;
			static const char* LOG_TAG;
			static const char* ATTITUDE_TAG;
			static const char* POSITION_TAG;

		private :
			boost::shared_ptr<Robot> robot;
			std::string fileName;
			json_t *jsonRoot;
			json_t *jsonStructure;
			json_t *jsonLog;
			std::vector<struct BodyDescriptor> bodies;

			WebGLLogger(const WebGLLogger& that); //disable copy constructor;
			const WebGLLogger& operator=(const WebGLLogger& that); //disable copy constructor
			void generateBodyCollection();
			void writeJSONHeaders();
			void writeRobotStructure();

	};
}

#endif

