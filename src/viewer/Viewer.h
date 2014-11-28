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
	~Viewer();
	bool configureScene(std::vector<boost::shared_ptr<Model> > bodyParts,
			boost::shared_ptr<Scenario> scenario);
	bool done();
	void frame();
	bool isPaused();

	void initializeRecording(std::string recordDirectoryName);
	void record();

private:
	osgViewer::Viewer *viewer;
	osg::ref_ptr<osg::Camera> camera;
	osg::ref_ptr<osg::Group> root;
	osg::ref_ptr<KeyboardHandler> keyboardEvent;
	unsigned int frameCount;
	std::string recordDirectoryName;

};

}

#endif /* VIEWER_H_ */
