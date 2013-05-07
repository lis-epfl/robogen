/*
 * @(#) KeyboardHandler.h   1.0   March 21, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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

namespace robogen {

class KeyboardHandler: public osgGA::GUIEventHandler {

public:

	KeyboardHandler() : osgGA::GUIEventHandler(), paused_(true), quit_(false) {

	}

	virtual bool handle(const osgGA::GUIEventAdapter& ea,
			osgGA::GUIActionAdapter&) {

		osgGA::GUIEventAdapter::EventType eventType = ea.getEventType();

		if (eventType == osgGA::GUIEventAdapter::KEYDOWN) {

			switch (ea.getKey()) {

			// Toggle Pause Simulation
			case 'p':
				if (paused_ == true) {
					paused_ = false;
				} else {
					paused_ = true;
				}
				return true;
				break;

			case 'q':
				if (quit_ == false) {
					quit_ = true;
				}
				return true;
				break;

			default:
				return false;

			}

		}

		return false;

	}

	virtual void accept(osgGA::GUIEventHandlerVisitor& v) {
		v.visit(*this);
	}

	/**
	 * Check if the pause button was pressed
	 */
	bool isPaused() {
		return paused_;
	}

	/**
	 * True if the quit button was pressed
	 */
	bool isQuit() {
		return quit_;
	}

private:

	bool paused_;

	bool quit_;

};

}

#endif /* ROBOGEN_KEYBOARD_HANDLER_H_ */
