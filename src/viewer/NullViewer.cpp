/*
 * @(#) FileViewerLog.h   1.0   Aug 20, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc
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

#include <viewer/NullViewer.h>

namespace robogen {

NullViewer::NullViewer() {
	// TODO Auto-generated constructor stub

}

NullViewer::~NullViewer() {
	// TODO Auto-generated destructor stub
}

} /* namespace robogen */

bool robogen::NullViewer::configureScene(
		std::vector<boost::shared_ptr<Model> > bodyParts,
		boost::shared_ptr<Scenario> scenario) {
	return true;
}

bool robogen::NullViewer::done() {
	return true;
}

bool robogen::NullViewer::frame(double simulatedTime,
		unsigned int numTimeSteps) {
	return true;
}

bool robogen::NullViewer::isPaused() {
	return true;
}
