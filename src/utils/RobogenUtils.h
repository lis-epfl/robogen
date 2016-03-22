/*
 * @(#) RobogenUtils.h   1.0   Feb 17, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_UTILS_H_
#define ROBOGEN_UTILS_H_

#include <boost/shared_ptr.hpp>
#include <osg/Quat>
#include <osg/Vec3>
#include <iostream>

#include "Models.h"
#include "RenderModels.h"
#include "robogen.pb.h"

#define MESH_DIRECTORY "../models/"

namespace robogen {

class RobogenUtils {

public:

	static const double OSG_EPSILON;
	static const double OSG_EPSILON_2;

	virtual ~RobogenUtils();

	static osg::Quat makeRotate(const osg::Vec3& from, const osg::Vec3& to);

	static bool areAxisParallel(const osg::Vec3& a, const osg::Vec3& b);

	static double getAngle(const osg::Vec3& a, const osg::Vec3& b);

	static std::istream& safeGetline(std::istream& is, std::string& t);

	/**
	 * It is b that remains fixed!!
	 */
	static boost::shared_ptr<Joint> connect(boost::shared_ptr<Model> a, unsigned int slotA,
			boost::shared_ptr<Model> b, unsigned int slotB,
			dJointGroupID connectionJointGroup, dWorldID odeWorld);

	static boost::shared_ptr<Model> createModel(
			const robogenMessage::BodyPart& bodyPart, dWorldID odeWorld,
			dSpaceID odeSpace);

	static boost::shared_ptr<RenderModel> createRenderModel(
			boost::shared_ptr<Model> model);

	static std::string getPartType(boost::shared_ptr<Model> model);

	static std::string getMeshFile(boost::shared_ptr<Model> model,
								const unsigned int id);
	static osg::Vec3 getRelativePosition(boost::shared_ptr<Model> model,
								const unsigned int id);
	static osg::Quat getRelativeAttitude(boost::shared_ptr<Model> model,
								const unsigned int id);

private:

	RobogenUtils();

};

}

#endif /* ROBOGEN_UTILS_H_ */
