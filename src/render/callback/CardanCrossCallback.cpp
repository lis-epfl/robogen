/*
 * @(#) CardanCrossCallback.cpp   1.0   Feb 13, 2013
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
#include <cmath>
#include <osg/PositionAttitudeTransform>

#include "model/components/CardanModel.h"
#include "render/callback/CardanCrossCallback.h"
#include "utils/RobogenUtils.h"

namespace robogen {

CardanCrossCallback::CardanCrossCallback(boost::shared_ptr<CardanModel> model) :
		model_(model) {

}

void CardanCrossCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {

	static const double CONNECTION_PART_OFFSET = fromOde(
			(CardanModel::CONNNECTION_PART_LENGTH / 2
					- (CardanModel::CONNNECTION_PART_LENGTH
							- CardanModel::CONNECTION_PART_OFFSET)));

	osg::ref_ptr<osg::PositionAttitudeTransform> pat =
			node->asTransform()->asPositionAttitudeTransform();

	osg::Quat attitudeConnectionA = model_->getBodyAttitude(
			CardanModel::B_CONNECTION_A_ID);

	osg::Quat attitudeConnectionB = model_->getBodyAttitude(
			CardanModel::B_CONNECTION_B_ID);

	osg::Vec3 posConnectionA = fromOde(
			model_->getBodyPosition(CardanModel::B_CONNECTION_A_ID));

	osg::Vec3 position(CONNECTION_PART_OFFSET, 0, 0);
	position = attitudeConnectionA * position;

	// Z vector of connection A
	osg::Vec3 zVectorConnectionA(0, 0, 1);
	zVectorConnectionA = attitudeConnectionA * zVectorConnectionA;

	// Z vector of connection B
	osg::Vec3 zVectorConnectionB(0, 0, 1);
	zVectorConnectionB = attitudeConnectionB * zVectorConnectionB;

	// Rotation between the two parts of the cardan
	osg::Quat rotation = RobogenUtils::makeRotate(zVectorConnectionA,
			zVectorConnectionB);

	pat->setAttitude(attitudeConnectionA * rotation);
	pat->setPosition(posConnectionA + position);

	traverse(node, nv);

}

}
