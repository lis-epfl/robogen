/*
 * @(#) CompositeBody.h   1.0   September 16, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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



#ifndef ROBOGEN_COMPOSITEBODY_H_
#define ROBOGEN_COMPOSITEBODY_H_


#include "Robogen.h"
#include <vector>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>



#include "AbstractBody.h"
#include "SimpleBody.h"

namespace robogen {

class CompositeBody : public AbstractBody {

public:
	//errorless constructor
	CompositeBody() { }

	void init(std::vector<boost::shared_ptr<AbstractBody> > subBodies,
			dWorldID world, bool multiModel = false);

	virtual ~CompositeBody();

	virtual osg::Vec3 getPosition();
	virtual osg::Quat getAttitude();

	inline const std::vector<boost::weak_ptr<AbstractBody> > &getSubBodies() {
		return subBodies_;
	}

	inline bool isMultiModel() {
		return multiModel_;
	}

	std::vector<boost::shared_ptr<Joint> > getAllJoints();

	std::string str(int indent=0);

private:

	void addSubBody(boost::shared_ptr<AbstractBody> subBody,
			bool directDescendant = true);
	void updateDescendantBodies();

	std::vector<boost::weak_ptr<SimpleBody> > flattenSubBodies();


	dMass compositeMass_;
	std::vector<boost::weak_ptr<AbstractBody> > subBodies_;
	bool multiModel_;
	std::set<dBodyID> bodiesToDestroy_;
	std::map<dGeomID, osg::Quat> rotations_;


};

}

#endif /* ROBOGEN_COMPOSITEBODY_H_ */


