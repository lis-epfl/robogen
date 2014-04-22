/*
 * @(#) Sensor.h   1.0   Feb 25, 2013
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
#ifndef ROBOGEN_LIGHT_SOURCE_H_
#define ROBOGEN_LIGHT_SOURCE_H_

#include <osg/Quat>
#include <osg/Vec3>
#include "model/PositionObservable.h"
#include "Robogen.h"

namespace robogen {

/**
 * A point immovable light source. Modeled as a sphere.
 */
class LightSource : public PositionObservable {

public:

	/**
	 * Light source radius
	 */
	static const float RADIUS;

	/**
	 * Initializes a LightSource
	 * @param position
	 * @param intensity A value >= 0
	 */
	LightSource(dSpaceID odeSpace, const osg::Vec3& position, float intensity);

	/**
	 * Destructor
	 */
	virtual ~LightSource();

	/**
	 * Inherited from PositionObservable
	 */
	virtual const osg::Vec3 getPosition();
	virtual const osg::Quat getAttitude();

	/**
	 * Sets the position of the light source
	 */
	void setPosition(const osg::Vec3& position);

	/**
	 * @return intensity
	 */
	float getIntensity();

	/**
	 * Sets the intensity of the light source
	 */
	void setIntensity(float intensity);

	/**
	 * @return the light source identifier
	 */
	dGeomID getSource();

private:

	/**
	 * Position of the light source
	 */
	osg::Vec3 position_;

	/**
	 * Intensity of the light source
	 */
	float intensity_;

	/**
	 * The light source
	 */
	dGeomID lightSource_;

};

}


#endif /* ROBOGEN_LIGHT_SOURCE_H_ */
