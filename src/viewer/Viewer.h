/*
 * @(#) Viewer.h   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
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


#ifndef VIEWER_H_
#define VIEWER_H_

#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include "viewer/KeyboardHandler.h"
#include "scenario/Scenario.h"
#include "model/Model.h"
#include "boost/date_time/posix_time/posix_time.hpp"

#define MAX_TIME_BETWEEN_FRAMES 0.05

namespace robogen{

/**
 * \brief Class for viewing a simulation
 *
 * Viewer wraps the OpenSceneGraph code, and should be used by other classes
 * that wish to visualize a simulation.
 *
 * Previously we had separate executables for Server, ServerViewer, and
 * FileViewer.  This involved a lot of duplicate code, so this class aims to be
 * a plugin to enable visualization or not.
 */
class Viewer{
public:
	Viewer(bool startPaused);
	Viewer(bool startPaused, double speedFactor);
	Viewer(bool startPaused, double speedFactor, bool recording,
			unsigned int recordFrequency, std::string recordDirectoryName);
	~Viewer();
	bool configureScene(std::vector<boost::shared_ptr<Model> > bodyParts,
			boost::shared_ptr<Scenario> scenario);
	bool done();

	/***
	 * frame:  updates frame if it should be updated
	 * params:
	 * 		simulatedTime: the amount of time simulated so far (in seconds)
	 * 		numTimeSteps: the number of time steps simulated so far
	 * returns:
	 * 		false if paused or going to fast
	 * 			(so simulator should continue without stepping physics)
	 * 		true otherwise
	 */

	bool frame(double simulatedTime, unsigned int numTimeSteps);

	bool isPaused();


private:
	void init(bool startPaused, double speedFactor, bool recording,
			unsigned int recordFrequency, std::string recordDirectoryName);
	void record();

	osgViewer::Viewer *viewer;
	osg::ref_ptr<osg::Camera> camera;
	osg::ref_ptr<osg::Group> root;
	osg::ref_ptr<KeyboardHandler> keyboardEvent;

	bool recording;
	unsigned int frameCount;
	std::string recordDirectoryName;
	unsigned int recordFrequency;

	boost::posix_time::ptime tick1, tick2;
	double elapsedWallTime;
	double speedFactor;
	double timeSinceLastFrame;

};

}

#endif /* VIEWER_H_ */
