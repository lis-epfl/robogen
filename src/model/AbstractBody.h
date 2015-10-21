/*
 * @(#) AbstractBody.h   1.0   September 13, 2015
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

#ifndef ROBOGEN_ABSTRACT_BODY_H_
#define ROBOGEN_ABSTRACT_BODY_H_

#include "Robogen.h"
#include <vector>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <boost/enable_shared_from_this.hpp>

#include "Joint.h"

namespace robogen {

class Model;
class CompositeBody;

class AbstractBody : public boost::enable_shared_from_this<AbstractBody> {

public:
	inline dBodyID getBody() { return body_; }
	inline void setBody(dBodyID body) { body_ = body; }
	inline virtual ~AbstractBody() {
		/*if(body_) {
			printf("destroying body!!!\n");
			std::cout << body_ << std::endl;
			dBodyDestroy(body_);
			body_ = NULL;
		}*/

	}


	virtual osg::Vec3 getPosition() = 0;
	virtual osg::Quat getAttitude() = 0;

	void setPosition(osg::Vec3 position);
	void setAttitude(osg::Quat attitude);

	inline void setParent(boost::shared_ptr<CompositeBody> parent) {
		parent_ = parent;
	}

	inline boost::shared_ptr<CompositeBody> getParent() {
		return parent_;
	}

	boost::shared_ptr<AbstractBody> getRoot();


protected:
	dBodyID body_;
	boost::shared_ptr<CompositeBody> parent_;
};

}

#endif /* ROBOGEN_ABSTRACT_BODY_H_ */
