/*
 * @(#) PositionObservable.h   1.0   Mar 20, 2013
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
#ifndef ROBOGEN_POSITION_OBSERVABLE_H_
#define ROBOGEN_POSITION_OBSERVABLE_H_

namespace robogen {

/**
 * Makes the position of an object/model observable
 */
class PositionObservable {

public:

	/**
	 * @return the position of the observed object
	 */
	virtual const osg::Vec3 getPosition() = 0;

	/**
	 * @return the attitude of the observed object
	 */
	virtual const osg::Quat getAttitude() = 0;


	/**
	 * Destructor
	 */
	virtual ~PositionObservable() {}

};

}


#endif /* ROBOGEN_POSITION_OBSERVABLE_H_ */
