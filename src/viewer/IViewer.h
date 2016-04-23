/*
 * @(#) Viewer.h   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc (guillaume.leclerc@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2014 Joshua Auerbach
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

#ifndef IVIEWER_H_
#define IVIEWER_H_

#include "scenario/Scenario.h"
#include "model/Model.h"
#include "boost/date_time/posix_time/posix_time.hpp"

#define MAX_TIME_BETWEEN_FRAMES 0.05
namespace robogen {
class IViewer {

public:
	virtual ~IViewer() {};
	virtual bool configureScene(std::vector<boost::shared_ptr<Model> > bodyParts,
			boost::shared_ptr<Scenario> scenario) = 0;
	virtual bool done() = 0;

	/***	 * frame:  updates frame if it should be updated	 * params:	 * 		simulatedTime: the amount of time simulated so far (in seconds)	 * 		numTimeSteps: the number of time steps simulated so far	 * returns:	 * 		false if paused or going to fast	 * 			(so simulator should continue without stepping physics)	 * 		true otherwise	 */

	virtual bool frame(double simulatedTime, unsigned int numTimeSteps) = 0;

	virtual bool isPaused() = 0;
};
}
#endif
