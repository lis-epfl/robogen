/*
 * @(#) LightSensor.cpp   1.0   Feb 25, 2013
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
#include "model/sensors/LightSensor.h"

#include <algorithm>
#include <iostream>
#include <cmath>

namespace robogen {

const float LightSensor::MIN_INTENSITY_VALUE = 0;
const float LightSensor::MAX_INTENSITY_VALUE = 1;
const double LightSensor::MIN_INTENSITY = 0;
const double LightSensor::MAX_INTENSITY = 100;
const double LightSensor::HALF_APERTURE = 15;

double LightSensor::lightIntensity(double angle, double distance){
	double intensity = 1;
	// first, angular dependency
	// Compare to VISHAY BPW85B data sheet: 0->1, 10->.9, 20->.7, 30->.4, 40->.1
	// in octave: polyfit([0 10 20 30 40], [1 .9 .7 .4 .1], 2)
	// returns: -3.5714e-04  -8.7143e-03   1.0086e+00
	angle *= 180./M_PI;
	intensity *= (1. - 0.0087143*angle - 0.00035714*angle*angle);
	// then, distance dependency
	// currently arbitrarily 1/r^2, saturation for r<1m
	if (distance > 1.){
		intensity /= (distance*distance);
	}
	return intensity;
}

LightSensor::LightSensor(dSpaceID odeSpace, std::vector<dBodyID> sensorBodies,
		std::string label):odeSpace_(odeSpace), label_(label),
		lastReadOutput_(MIN_INTENSITY_VALUE){
	for(unsigned int i=0; i<sensorBodies.size(); i++){
		for (dGeomID g=dBodyGetFirstGeom(sensorBodies[i]); g
				; g=dBodyGetNextGeom(g)){
			sensorGeoms_.push_back(g);
		}
	}
	raySpace_ = dHashSpaceCreate(0);
}

std::string &LightSensor::getLabel(){
	return label_;
}

LightSensor::~LightSensor() {
	dSpaceDestroy(raySpace_);
}

void LightSensor::update(const osg::Vec3& position, const osg::Quat& attitude) {
	this->position_ = position;
	this->attitude_ = attitude;
}

struct RayTrace {
	std::vector<dGeomID> ignoreGeoms;
	bool visible;
};

void LightSensor::collisionCallback(void *data, dGeomID o1, dGeomID o2){
	RayTrace *r = (RayTrace*) data;
	// ignore what needs ignoring
	if (std::find(r->ignoreGeoms.begin(),r->ignoreGeoms.end(),
			o1) != r->ignoreGeoms.end() ||
			std::find(r->ignoreGeoms.begin(),r->ignoreGeoms.end(),
					o2) != r->ignoreGeoms.end()){
		return;
	}
	// not visible if colliding with anything else
	dContact contact;
	if (dCollide(o1,o2,1,&contact.geom,sizeof(contact))){
		r->visible = false;
	}
}

float LightSensor::read(
		const std::vector<boost::shared_ptr<LightSource> >& lightSources,
		double ambientLight) {

	// add ambient light anyways
	ambientLight = 0; //TODO temporary
	float totalLight = ambientLight;

	// For each light source, we trace a ray to the given sensor
	for (unsigned int i=0; i<lightSources.size(); i++){
		// calculations
		osg::Vec3 lightSourcePos = lightSources[i]->getPosition();
		osg::Vec3 sensorToLight = lightSourcePos - position_;
		// create ray
		dGeomID ray = dCreateRay(raySpace_, sensorToLight.length());
		// position ray
		dGeomRaySet(ray, position_.x(), position_.y(), position_.z(),
				sensorToLight.x(), sensorToLight.y(), sensorToLight.z());
		// abort if angle > cutoff
		osg::Vec3 sensorRel = attitude_.inverse() * sensorToLight;
		sensorRel.normalize();
		double angle = acos(sensorRel.x());
		if (angle*180/M_PI < 40){
			// prepare collision data structure
			RayTrace data;
			data.visible = true;
			// ignore sensor bodies and light source body
			data.ignoreGeoms.insert(data.ignoreGeoms.end(),
					sensorGeoms_.begin(), sensorGeoms_.end());
			data.ignoreGeoms.push_back(lightSources[i]->getSource());
			// perform collision
			dSpaceCollide2((dGeomID)raySpace_, (dGeomID)odeSpace_, (void*)&data,
					LightSensor::collisionCallback);
			// calculate intensity from angle if visible
			if (data.visible){
				totalLight += lightIntensity(angle, sensorToLight.length());
			}
		}
		dGeomDestroy(ray);
	}


	totalLight = (totalLight>1.)?1.:totalLight;
	return totalLight;
}

}
