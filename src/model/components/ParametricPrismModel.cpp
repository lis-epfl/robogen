/*
 * @(#) ParametricPrismModel.cpp   1.0   Oct 5, 2016
 *
 * Gaël Gorret (gael.gorret@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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

#include "model/components/ParametricPrismModel.h"
#include "model/ConvexBody.h"

namespace robogen {
/************************************************************
*	TODO: 	the constance MASS_PRISM is wrong so complet the
*			funtion ParametricPrismModel::computePolygon_dMass
*
*************************************************************/
	const float ParametricPrismModel::MASS_PRISM = inGrams(20);
	const float ParametricPrismModel::MASS_CORE = MASS_PRISM + inGrams(34.3);
	const float ParametricPrismModel::WIDTHY = inMm(41);
	const float ParametricPrismModel::HEIGHTZ = inMm(35.5);
	const float ParametricPrismModel::SLOT_THICKNESS = inMm(1.5);

	ParametricPrismModel::ParametricPrismModel(dWorldID odeWorld, dSpaceID odeSpace,
			std::string id, int faceNumber, bool isCore, bool hasSensors):
			PerceptiveComponent(odeWorld, odeSpace, id),
			faceNumber_(faceNumber), isCore_(isCore), hasSensors_(hasSensors){
				
				if (hasSensors) {
					sensor_.reset(new ImuSensor(id + "-IMU"));
				}

				float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);

				ParametricPrismModel::distanceFaceCenter_ = 0.5*WIDTHY/(tan(PrismeFaceAngle/2.0));
				ParametricPrismModel::initialisationDone_ = false;
				ParametricPrismModel::topFaceSlotId_ = faceNumber_;
				ParametricPrismModel::bottomFaceSlotId_ = faceNumber_ + 1;
	}

	ParametricPrismModel::~ParametricPrismModel() {

	}

/**********************************************************************
*                   Initialisation functions
***********************************************************************/

	bool ParametricPrismModel::initModel(){
		std::vector <dReal> pointsVector;
		std::vector <dReal> planeVector;
		std::vector <unsigned int> polygonVector;
		dMass massOde;

		const unsigned int pointCount = 2*faceNumber_;
		const unsigned int planeCount = faceNumber_ + 2;
		
		planeVector = constructPlaneVector();
		pointsVector = constructPointsVector();
		polygonVector = constructPolygonVector();
		massOde = computePolygon_dMass();

		Root_ = this -> addConvex(massOde,osg::Vec3(0,0,0),  
					pointCount, &pointsVector[0], 
					planeCount, &planeVector[0], 
					&polygonVector[0], 0);
		ParametricPrismModel::initialisationDone_ = true;

		return true;
	}

/* create a mass for the ConvexPolygon
 * Currently we use a cylinder as approx of prism
 * TODO: Implement the exact moment of inertia and his mass for a general prism
 *		 Create a table with the weight related with the number of faces
 */
	dMass ParametricPrismModel::computePolygon_dMass(){
	    // To use density instead of mass, use the following function
		//void dMassSetCylinder (&massOde, inGrams(), int direction, dReal radius, dReal length);
	    dMass massOde;
	    if(isCore_)
	    	dMassSetCylinderTotal (&massOde, MASS_CORE, 3, distanceFaceCenter_, HEIGHTZ);
	    else	
	    	dMassSetCylinderTotal (&massOde, MASS_PRISM, 3, distanceFaceCenter_, HEIGHTZ);
	   	return massOde;
	}

/*Create an Array with the normal vector of each face (X,Y,Z) with its
 *distance to the origine D. So 4 parameters in order to describe one vector
 *(X,Y,Z,D) that follow the equation : Xx + Yy + Zz + D = 0
 */ 
	std::vector <dReal> ParametricPrismModel::constructPlaneVector(){
		std::vector <dReal> planeVector;

		//create plane array
		for(int i = 0; i<faceNumber_; i++){
			osg::Vec3 normal;
			normal = this -> getSlotAxis(i);
			planeVector.push_back(normal.x());
			planeVector.push_back(normal.y());
			planeVector.push_back(normal.z());
			planeVector.push_back(distanceFaceCenter_);
		}
		//add the vector describing the top face
			planeVector.push_back(0);
			planeVector.push_back(0);
			planeVector.push_back(1);
			planeVector.push_back(HEIGHTZ/2.0f);
		//add the vector describing the top face
			planeVector.push_back(0);
			planeVector.push_back(0);
			planeVector.push_back(-1);
			planeVector.push_back(HEIGHTZ/2.0f);

		return planeVector;	
	}

/* An array of indices to the points of each polygon,
 * it should be the number of vertices followed by 
 * that amount of indices to "points" in counter clockwise order 
 */
	std::vector <unsigned int> ParametricPrismModel::constructPolygonVector(){
		std::vector <unsigned int> polygonVector;

		//compute for the square face of the prism
		for(int i = 0; i<faceNumber_; i++){
			if(i!=faceNumber_-1){
				polygonVector.push_back(4);
				polygonVector.push_back(i);
				polygonVector.push_back(i+1);
				polygonVector.push_back(faceNumber_ + i+1);
				polygonVector.push_back(faceNumber_ + i);
			}
		//compute the last square face
			else
			{
				polygonVector.push_back(4);
				polygonVector.push_back(i);
				polygonVector.push_back(0);
				polygonVector.push_back(faceNumber_);
				polygonVector.push_back(faceNumber_ + i);
			}
		}
		//compute the bottom face
		polygonVector.push_back(faceNumber_);
		for(int i = 0; i<faceNumber_; i++){
			polygonVector.push_back((faceNumber_ - 1) - i);
		}
		//compute top face
		polygonVector.push_back(faceNumber_);
		for(int i = 0; i<faceNumber_; i++){
			polygonVector.push_back(faceNumber_ + i);
		}
		return polygonVector;
	}
/* Create arrays for ConvexPolygon
* An array of points X,Y,Z that define coordinates of each corner
*/
std::vector <dReal> ParametricPrismModel::constructPointsVector(){
	std::vector <dReal> pointsVector;
	osg::Vec3 normal;
	osg::Vec3 tangent;
	osg::Vec3 bottomPointPosition;
	osg::Vec3 topPointPosition;
	osg::Vec3 zAxis = osg::Vec3(0, 0, 1);
	
	//compute the coordinate of the bottom corner
	for(int i = 0; i < faceNumber_; i++){		
		normal  = this -> getSlotAxis(i);
		tangent = this -> getSlotOrientation(i);

		bottomPointPosition = normal*distanceFaceCenter_ - tangent*WIDTHY/2.0f - zAxis*HEIGHTZ/2.0f;

		pointsVector.push_back(bottomPointPosition.x());
		pointsVector.push_back(bottomPointPosition.y());
		pointsVector.push_back(bottomPointPosition.z());
	}
	//compute the coordinate of the top corner
	for(int i = 0; i < faceNumber_; i++){		
		normal  = this -> getSlotAxis(i);
		tangent = this -> getSlotOrientation(i);

		topPointPosition    = normal*distanceFaceCenter_ - tangent*WIDTHY/2.0f + zAxis*HEIGHTZ/2.0f;

		pointsVector.push_back(topPointPosition.x());
		pointsVector.push_back(topPointPosition.y());
		pointsVector.push_back(topPointPosition.z());
	}

	return pointsVector;
}
/**********************************************************************
*                       Model functions
***********************************************************************/
	boost::shared_ptr<SimpleBody> ParametricPrismModel::getRoot() {
		return Root_;
	}

	boost::shared_ptr<SimpleBody> ParametricPrismModel::getSlot(unsigned int i) {
		return Root_;
	}

	//get the position of the faces
	osg::Vec3 ParametricPrismModel::getSlotPosition(unsigned int i) {
		if (i >= faceNumber_) {
			std::cout << "[ParametricPrismModel] Invalid slot: " << i << std::endl;
			assert(i < faceNumber_);
		}

		osg::Vec3 curPos = this->getRootPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i) *
			(distanceFaceCenter_ - SLOT_THICKNESS);

		curPos = curPos + slotAxis;

		return curPos;
	}

// return the normal vector of the face asked
	osg::Vec3 ParametricPrismModel::getSlotAxis(unsigned int i) {
		if (i >= faceNumber_) {
			std::cout << "[ParametricPrismModel] Invalid slot: " << i << std::endl;
			assert(i < faceNumber_);
		}

		osg::Quat quat;
		osg::Vec3 axis;
		osg::Quat rotation;

		if(initialisationDone_)
			quat = this->getRootAttitude();
			
			if (i < faceNumber_){
				axis.set(1,0,0); //normal vector of the first face.

				float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);
				rotation.makeRotate(i*PrismeFaceAngle, osg::Vec3(0, 0, 1));
			}		
		#ifndef ENFORCE_PLANAR
			else if (i == topFaceSlotId_) {

				// Top face
				axis.set(0, 0, 1);
				rotation.makeRotate(0, osg::Vec3(0, 0, 1));

			} else if (i == bottomFaceSlotId_) {

				// Bottom face
				axis.set(0, 0, -1);
				rotation.makeRotate(0, osg::Vec3(0, 0, 1));
			}
		#endif

		if(initialisationDone_)
			return quat * rotation * axis;
		return rotation * axis;
	}

// return the orientation vector of the face asked
	osg::Vec3 ParametricPrismModel::getSlotOrientation(unsigned int i) {
		if (i >= faceNumber_) {
			std::cout << "[ParametricPrismModel] Invalid slot: " << i << std::endl;
			assert(i < faceNumber_);
		}

		osg::Quat quat;
		osg::Vec3 tangent;
		osg::Quat rotation;

		if(initialisationDone_)
			quat = this->getRootAttitude();
			
			if (i < faceNumber_){
				tangent.set(0,1,0); //tangent vector of the first face.

				float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);
				rotation.makeRotate(i*PrismeFaceAngle, osg::Vec3(0, 0, 1));
			}		
		#ifndef ENFORCE_PLANAR
			else if (i == topFaceSlotId_) {

				// Top face
				tangent.set(0, 0, 1);
				rotation.makeRotate(0, osg::Vec3(1, 0, 0));

			} else if (i == bottomFaceSlotId_) {

				// Bottom face
				tangent.set(0, 0, -1);
				rotation.makeRotate(0, osg::Vec3(1, 0, 0));
			}
		#endif

		if(initialisationDone_)
			return quat * rotation * tangent;
		return rotation * tangent;
	}
	
	void ParametricPrismModel::getSensors(
		std::vector<boost::shared_ptr<Sensor> >& sensors) {
		if (sensor_ != NULL) {
			sensor_->getSensors(sensors);
		}
	}

	void ParametricPrismModel::updateSensors(boost::shared_ptr<Environment>& env) {
		if (sensor_ != NULL) {
			dVector3 gravity;
			dWorldGetGravity(getPhysicsWorld(), gravity);
			sensor_->update(this->getRootPosition(), this->getRootAttitude(),
					env->getTimeElapsed(),
					osg::Vec3(gravity[0], gravity[1], gravity[2]));
		}
	}
}