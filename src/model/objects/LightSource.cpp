/*
 * @(#) LightSource.cpp   1.0   Feb 25, 2013
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
#include "model/objects/LightSource.h"

namespace robogen {

const float LightSource::RADIUS = 0.02;

LightSource::LightSource(dSpaceID odeSpace, const osg::Vec3& position,
		float intensity) :
		position_(position), intensity_(intensity) {

	lightSource_ = dCreateSphere(odeSpace, RADIUS);
	dGeomSetPosition(lightSource_, position.x(), position.y(), position.z());
}

LightSource::~LightSource() {

}

const osg::Vec3 LightSource::getPosition() {
	return position_;
}

void LightSource::setPosition(const osg::Vec3& position) {
	this->position_ = position;
	dGeomSetPosition(lightSource_, position.x(), position.y(), position.z());
}

const osg::Quat LightSource::getAttitude() {
	return osg::Quat();
}

float LightSource::getIntensity() {
	return intensity_;
}

void LightSource::setIntensity(float intensity) {
	this->intensity_ = intensity;
}

dGeomID LightSource::getSource() {
	return lightSource_;
}

}
