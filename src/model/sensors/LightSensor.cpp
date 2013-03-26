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

#include <map>

namespace robogen {

const int LightSensor::MIN_INTENSITY_VALUE = 0;
const int LightSensor::MAX_INTENSITY_VALUE = 1023;
const double LightSensor::MIN_INTENSITY = 0;
const double LightSensor::MAX_INTENSITY = 100;
const double LightSensor::HALF_APERTURE = 10;
const double LightSensor::SENSOR_RESOLUTION = 5;

LightSensor::LightSensor(dSpaceID odeSpace) :
		odeSpace_(odeSpace) {

}

LightSensor::~LightSensor() {

}

void LightSensor::update(const osg::Vec3& position, const osg::Quat& attitude) {
	this->position_ = position;
	this->attitude_ = attitude;

}

//-----------------------------------------------
// Ray-tracing
//-----------------------------------------------

struct RayTrace {
	int lightSourceId;
	double azimuth;
	double altitude;
	std::vector<bool> visible;
	std::vector<bool> objectsInBetween;
	std::vector<double> collisions;
	std::vector<double> distanceToLight;
	dGeomID geometry;
};

struct LightSourceInfo {
	int id;
};

void odeRayTracingCallback(void*, dGeomID o1, dGeomID o2) {

	const int MAX_CONTACTS = 1; // maximum number of contact points per body

	// Here we intersect only bodies and rays or rays and light sources (at least one with bodyId == 0)
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if (b1 != 0 && b2 != 0) {
		return;
	}

	dContact contact[MAX_CONTACTS];
	/*for (int i = 0; i < MAX_CONTACTS; i++) {

		contact[i].surface.mode = dContactBounce | dContactSoftCFM;
		contact[i].surface.mu = dInfinity;
		contact[i].surface.mu2 = 0;
		contact[i].surface.bounce = 0.01;
		contact[i].surface.bounce_vel = 0.1;
		contact[i].surface.soft_cfm = 0.0001;

	}*/

	// Check what kind of geometries
	CustomGeomData* userDataO1 = (CustomGeomData*) dGeomGetData(o1);
	CustomGeomData* userDataO2 = (CustomGeomData*) dGeomGetData(o2);

	bool isLightSourceO1 = false;
	if (userDataO1 != NULL
			&& userDataO1->getId() == CustomGeomData::LIGHT_SOURCE_INFO) {
		isLightSourceO1 = true;
	}

	bool isLightSourceO2 = false;
	if (userDataO2 != NULL
			&& userDataO2->getId() == CustomGeomData::LIGHT_SOURCE_INFO) {
		isLightSourceO2 = true;
	}

	bool isRayO1 = false;
	if (userDataO1 != NULL
			&& userDataO1->getId() == CustomGeomData::RAY_TRACE_INFO) {
		isRayO1 = true;
	}

	bool isRayO2 = false;
	if (userDataO2 != NULL
			&& userDataO2->getId() == CustomGeomData::RAY_TRACE_INFO) {
		isRayO2 = true;
	}

	if (!(isRayO1 && isRayO2) && !(isLightSourceO1 && isLightSourceO2)) {

		int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
				sizeof(dContact));

		if (collisionCounts != 0) {

			if (b1 == 0 && b2 == 0) {

				if (isLightSourceO1 && isRayO2) {

					LightSourceInfo* ls =
							(LightSourceInfo*) userDataO1->getData();
					RayTrace* ray = (RayTrace*) userDataO2->getData();

					ray->visible[ls->id] = true;
					ray->distanceToLight[ls->id] = contact[0].geom.depth;

					/*std::cout << "LightSource(1) VS Ray[" << ray->azimuth
					 << "° - " << ray->altitude << "°](2) ["
					 << ray->visible[ls->id] << ", "
					 << ray->distanceToLight[ls->id] << "]" << std::endl;*/

				} else if (isRayO1 && isLightSourceO2) {

					LightSourceInfo* ls =
							(LightSourceInfo*) userDataO2->getData();
					RayTrace* ray = (RayTrace*) userDataO1->getData();

					ray->visible[ls->id] = true;
					ray->distanceToLight[ls->id] = contact[0].geom.depth;

					/*std::cout << "LightSource(2) VS Ray[" << ray->azimuth
					 << "° - " << ray->altitude << "°](1) ["
					 << ray->visible[ls->id] << ", "
					 << ray->distanceToLight[ls->id] << "]" << std::endl;*/

				} else if (isRayO1 && !isLightSourceO2) {

					RayTrace* ray = (RayTrace*) userDataO1->getData();
					ray->collisions.push_back(contact[0].geom.depth);

					/*std::cout << "Ray[" << ray->azimuth << "° - "
					 << ray->altitude << "°](1) VS Obj(2) [Collisions: ";
					 for (unsigned int i = 0; i < ray->collisions.size(); ++i) {
					 std::cout << ray->collisions[i];
					 if (i > 0) {
					 std::cout << ", ";
					 }
					 }
					 std::cout << "]" << std::endl;*/

				} else if (isRayO2 && !isLightSourceO1) {

					RayTrace* ray = (RayTrace*) userDataO2->getData();
					ray->collisions.push_back(contact[0].geom.depth);

					/*std::cout << "Ray[" << ray->azimuth << "° - "
					 << ray->altitude << "°](2) VS Obj(1) [Collisions: ";
					 for (unsigned int i = 0; i < ray->collisions.size(); ++i) {
					 std::cout << ray->collisions[i];
					 if (i > 0) {
					 std::cout << ", ";
					 }
					 }
					 std::cout << "]" << std::endl;*/

				}

			}

		}

		/*else {
		 if (isRayO1) {

		 RayTrace* ray = (RayTrace*) userDataO1->getData();
		 std::cout << "Ray[" << ray->azimuth << "° - " << ray->altitude
		 << "°]" << std::endl;
		 } else if (isRayO2) {

		 RayTrace* ray = (RayTrace*) userDataO2->getData();
		 std::cout << "Ray[" << ray->azimuth << "° - " << ray->altitude
		 << "°]" << std::endl;
		 }

		 }*/

	}

}

int LightSensor::read(
		const std::vector<boost::shared_ptr<LightSource> >& lightSources,
		double ambientLight) {

	std::vector<boost::shared_ptr<LightSourceInfo> > lightSourceInfos;
	std::vector<boost::shared_ptr<RayTrace> > rayGeometries;

	double totalLight = ambientLight;
	for (unsigned int i = 0; i < lightSources.size(); ++i) {

		boost::shared_ptr<LightSourceInfo> lightInfo(new LightSourceInfo);
		lightInfo->id = i;
		dGeomSetData(lightSources[i]->getSource(),
				new CustomGeomData(CustomGeomData::LIGHT_SOURCE_INFO,
						lightInfo.get()));
		lightSourceInfos.push_back(lightInfo);

	}

	// We need to check if the light emitted by sources collides with the sensor

	// 1. Generate ray geometries that will be used to test against the light source
	for (double altitude = -HALF_APERTURE; altitude <= HALF_APERTURE;
			altitude += SENSOR_RESOLUTION) {

		osg::Quat rayQuatAltitude;
		rayQuatAltitude.makeRotate(osg::inDegrees((90 + altitude)),
				osg::Vec3(0, 1, 0));

		for (double azimuth = 0; azimuth < 360; azimuth += SENSOR_RESOLUTION) {

			// Generate a ray with origin in the sensor position, passing at a point at specified altitude and azimut
			osg::Quat rayQuatAzimuth;
			rayQuatAzimuth.makeRotate(osg::inDegrees(azimuth),
					osg::Vec3(0, 0, 1));

			osg::Quat curQuat = attitude_ * rayQuatAltitude * rayQuatAzimuth;

			dGeomID curRay = dCreateRay(odeSpace_, 100000); // A very long ray
			dGeomSetPosition(curRay, position_.x(), position_.y(),
					position_.z());
			dQuaternion quatOde;
			quatOde[0] = curQuat.w();
			quatOde[1] = curQuat.x();
			quatOde[2] = curQuat.y();
			quatOde[3] = curQuat.z();
			dGeomSetQuaternion(curRay, quatOde);

			boost::shared_ptr<RayTrace> rayTrace(new RayTrace);

			rayTrace->objectsInBetween.resize(lightSources.size(), false);
			rayTrace->visible.resize(lightSources.size(), false);
			rayTrace->distanceToLight.resize(lightSources.size(), false);
			rayTrace->azimuth = azimuth;
			rayTrace->altitude = altitude;
			rayTrace->geometry = curRay;

			dGeomSetData(curRay,
					new CustomGeomData(CustomGeomData::RAY_TRACE_INFO,
							rayTrace.get()));
			rayGeometries.push_back(rayTrace);

		}

	}

	// RayTracing
	dSpaceCollide(odeSpace_, 0, odeRayTracingCallback);

	// Compute light received by sensor from each light source
	for (unsigned int i = 0; i < rayGeometries.size(); ++i) {

		for (unsigned int j = 0; j < lightSources.size(); ++j) {
			if (rayGeometries[i]->visible[j]) {

				//std::cout << "Ray: " << i << " (" << rayGeometries[i]->azimuth << ", " << rayGeometries[i]->altitude << ") Coll: [";

				// Check if there is an object between sensor and light source
				bool objectsInBetween = false;
				std::sort(rayGeometries[i]->collisions.begin(),
						rayGeometries[i]->collisions.end());
				for (unsigned int k = 0;
						k < rayGeometries[i]->collisions.size(); ++k) {

					//std::cout << rayGeometries[i]->collisions[k] << " ";

					if (rayGeometries[i]->collisions[k]
							< rayGeometries[i]->distanceToLight[j]) {
						objectsInBetween = true;
						break;
					}
				}

				//std::cout << "], DL: " << rayGeometries[i]->distanceToLight[j] << std::endl;

				if (!objectsInBetween) {
					totalLight += lightSources[j]->getIntensity();
				}

			}

		}
	}

	// Clear rays
	for (unsigned int i = 0; i < rayGeometries.size(); ++i) {

		CustomGeomData* customData = (CustomGeomData*) dGeomGetData(
				rayGeometries[i]->geometry);
		delete customData;

		dGeomDestroy(rayGeometries[i]->geometry);
	}

	// Clear light source
	for (unsigned int i = 0; i < lightSources.size(); ++i) {

		CustomGeomData* customData = (CustomGeomData*) dGeomGetData(
				lightSources[i]->getSource());
		delete customData;

		dGeomSetData(lightSources[i]->getSource(), NULL);
	}

	// Rescale total light received
	if (totalLight >= MAX_INTENSITY) {
		return MAX_INTENSITY_VALUE;
	} else if (totalLight <= MIN_INTENSITY) {
		return MIN_INTENSITY_VALUE;
	} else {
		return MIN_INTENSITY_VALUE
				+ (std::min(totalLight, MAX_INTENSITY) - MIN_INTENSITY) / 100
						* (MAX_INTENSITY_VALUE - MIN_INTENSITY_VALUE);
	}

}

}
