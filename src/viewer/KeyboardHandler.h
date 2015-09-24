/*
 * @(#) KeyboardHandler.h   1.0   March 21, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_KEYBOARD_HANDLER_H_
#define ROBOGEN_KEYBOARD_HANDLER_H_

#include <osgGA/GUIEventHandler>
#include <osg/Version>

namespace robogen {

class KeyboardHandler: public osgGA::GUIEventHandler {

public:

	KeyboardHandler(bool startPaused, bool geoms, bool meshes/*, bool transparent*/)
		: osgGA::GUIEventHandler(), paused_(startPaused), geoms_(geoms),
		  meshes_(meshes), /*transparent_(transparent),*/ quit_(false)  {

	}

	virtual bool handle(const osgGA::GUIEventAdapter& ea,
			osgGA::GUIActionAdapter&) {

		osgGA::GUIEventAdapter::EventType eventType = ea.getEventType();

		if (eventType == osgGA::GUIEventAdapter::KEYDOWN) {

			switch (ea.getKey()) {

			// Toggle Pause Simulation
			case 'p':
				paused_ = !paused_;
				return true;
				break;

			case 'q':
				quit_ = true;
				return true;
				break;

			case 'g':
				geoms_ = !geoms_;
				return true;
				break;

			case 'm':
				meshes_ = !meshes_;
				return true;
				break;

			//case 't':
			//	transparent_ = !transparent_;
			//	return true;
			//	break;

			default:
				return false;

			}

		}

		return false;

	}
#if OSG_VERSION_LESS_OR_EQUAL(3, 2, 0)
	//think this can safely be removed complete, but will wrap in this check
	// for now
	virtual void accept(osgGA::GUIEventHandlerVisitor& v) {
		v.visit(*this);
	}
#endif

	/**
	 * Check if the pause button was pressed
	 */
	bool isPaused() {
		return paused_;
	}

	bool showGeoms() {
		return geoms_;
	}

	bool showMeshes() {
		return meshes_;
	}

	//bool isTransparent() {
	//	return transparent_;
	//}


	/**
	 * True if the quit button was pressed
	 */
	bool isQuit() {
		return quit_;
	}

private:

	bool paused_;

	bool geoms_;

	bool meshes_;

	//bool transparent_;

	bool quit_;

};

}

#endif /* ROBOGEN_KEYBOARD_HANDLER_H_ */
