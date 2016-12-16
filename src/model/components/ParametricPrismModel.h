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

#ifndef ROBOGEN_PARAMETRIC_PRISM_MODEL_H_
#define ROBOGEN_PARAMETRIC_PRISM_MODEL_H_

//#include "model/Model.h"
#include "model/PerceptiveComponent.h"
#include "model/sensors/ImuSensor.h"
#include "PartList.h"

namespace robogen {
/**
 * A parametric prism is modelled with ConvexBody
 */
	class ParametricPrismModel: public PerceptiveComponent {

		public:

			static const float MASS_PRISM;
			static const float MASS_CORE;
			static const float WIDTHY;
			static const float HEIGHTZ;
			static const float SLOT_THICKNESS;

			/**
			 * Initialize a parametric Prism model
			 *
			 * @param odeWorld
			 * @param odeSpace
			 * @param FaceNumber = number of prism face (without counting the face on the top and the bottom)
			 */
			ParametricPrismModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id,
					int faceNumber, bool isCore, bool hasSensors);

			virtual ~ParametricPrismModel();

			virtual bool initModel();

			virtual boost::shared_ptr<SimpleBody> getRoot();

			virtual boost::shared_ptr<SimpleBody> getSlot(unsigned int i);

			virtual osg::Vec3 getSlotPosition(unsigned int i);

			virtual osg::Vec3 getSlotOrientation(unsigned int i);

			virtual osg::Vec3 getSlotAxis(unsigned int i);

			virtual void getSensors(std::vector<boost::shared_ptr<Sensor> >& sensors);

			virtual void updateSensors(boost::shared_ptr<Environment>& env);

			inline int getFaceNumber() {
				return faceNumber_;
			}

			inline int getTopFaceSlotId() {
				return topFaceSlotId_;
			}

			inline int getbottomFaceSlotId() {
				return bottomFaceSlotId_;
			}

			inline float getDistanceFaceCenter(){
				return distanceFaceCenter_;
			}

			inline bool isCore() {
				return isCore_;
			}

			inline bool hasSensors() {
				return hasSensors_;
			}

		private:

			int faceNumber_;
			int topFaceSlotId_;
			int bottomFaceSlotId_;

			float distanceFaceCenter_;
			
			bool initialisationDone_; //I can't put a default value on the function declared in Model.h
			bool isCore_;
			bool hasSensors_;

			boost::shared_ptr<ImuSensor> sensor_;
			boost::shared_ptr<SimpleBody> Root_;

			std::vector <dReal> constructPlaneVector();
			std::vector <unsigned int> constructPolygonVector();
			std::vector <dReal> constructPointsVector();
			dMass computePolygon_dMass();
	};
}
#endif /* ROBOGEN_PARAMETRIC_PRISM_MODEL_H_ */