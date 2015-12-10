/*
 * @(#) LightSensor.cpp   1.0   Feb 25, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani, Joshua Auerbach
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

//TODO - Are these used at all?  Remove
const float LightSensor::MIN_INTENSITY_VALUE = 0;
const float LightSensor::MAX_INTENSITY_VALUE = 1;
const double LightSensor::MIN_INTENSITY = 0;
const double LightSensor::MAX_INTENSITY = 100;
const double LightSensor::HALF_APERTURE = 15;

double LightSensor::getIntensity(double angle, double lightIntensity,
		double distance){
	double intensity = lightIntensity;
	// first, angular dependency
	// Compare to VISHAY BPW85B data sheet: 0->1, 10->.9, 20->.7, 30->.4, 40->.1
	// in octave: polyfit([0 10 20 30 40], [1 .9 .7 .4 .1], 2)
	// returns: -3.5714e-04  -8.7143e-03   1.0086e+00
	angle *= 180./M_PI;
	//intensity *= (1.0086 - 0.0087143*angle - 0.00035714*angle*angle);

	// hardware tests showed a different response pattern, possibly due to the
	// plastic cylinder around the sensor?  TODO: more tests to verify
	// this response patter appears much more logistic
	// fit a logistic function to data
	// measured at .5 meter: 0->0.72, 10->0.66, 20->0.26, 30->0.06, 40->0.05
	// 1.0 / (1.0 + exp(0.26530016 * angle - 4.80599073))
	intensity *= (1.0 / (1.0 + exp(0.26530016 * angle - 4.80599073)));



	// then, distance dependency
	// based on fitted inverse square to hardware test
	// [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.5, 2]
	// -> [.98, .7, .53, .4, .3, .25, .2, .17, .15, .13, .11, .08]
	//
	// 0.15634027 / (distance^2)

	double distanceFactor = 0.15634027 / (distance * distance);
	// threshold at 1, so won't saturate sensor if at an angle
	if (distanceFactor < 1.0)
		intensity *= distanceFactor;
	return intensity;
}

LightSensor::LightSensor(dSpaceID odeSpace,
		std::vector<boost::shared_ptr<SimpleBody> > sensorBodies,
		std::string label):Sensor(label), odeSpace_(odeSpace),
		lastReadOutput_(MIN_INTENSITY_VALUE){
	for(unsigned int i=0; i<sensorBodies.size(); i++){
		for (dGeomID g=dBodyGetFirstGeom(sensorBodies[i]->getBody()); g
				; g=dBodyGetNextGeom(g)){
			sensorGeoms_.push_back(g);
		}
	}
	raySpace_ = dHashSpaceCreate(0);
}

LightSensor::~LightSensor() {
	dSpaceDestroy(raySpace_);
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

void LightSensor::update(const osg::Vec3& position, const osg::Quat& attitude,
		boost::shared_ptr<Environment> env) {
	this->position_ = position;
	this->attitude_ = attitude;


	// add ambient light anyways
	float totalLight = env->getAmbientLight();
	// For each light source, we trace a ray to the given sensor
	for (unsigned int i=0; i<env->getLightSources().size(); i++){
		// calculations
		osg::Vec3 lightSourcePos = env->getLightSources()[i]->getPosition();
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
		if (angle*180/M_PI < 42){
			// prepare collision data structure
			RayTrace data;
			data.visible = true;
			// ignore sensor bodies and light source body
			data.ignoreGeoms.insert(data.ignoreGeoms.end(),
					sensorGeoms_.begin(), sensorGeoms_.end());
			data.ignoreGeoms.push_back(env->getLightSources()[i]->getSource());
			// perform collision
			dSpaceCollide2((dGeomID)raySpace_, (dGeomID)odeSpace_, (void*)&data,
					LightSensor::collisionCallback);
			// calculate intensity from angle if visible
			if (data.visible){
				totalLight += getIntensity(angle,
						env->getLightSources()[i]->getIntensity(),
						sensorToLight.length());
			}
		}
		dGeomDestroy(ray);
	}


	totalLight = (totalLight>1.)?1.:totalLight;
	updateValue( totalLight );
}

}
