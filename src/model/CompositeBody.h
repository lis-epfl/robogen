/*
 * CompositeBody.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: auerbach
 */

#ifndef ROBOGEN_COMPOSITEBODY_CPP_
#define ROBOGEN_COMPOSITEBODY_CPP_


#include "Robogen.h"
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include "PhysicalBody.h"
#include "SimpleBody.h"

namespace robogen {

class CompositeBody : public PhysicalBody {

public:
	/*
	 * This constructor will create a new CompositeBody by merging two existing
	 * bodies
	 */
	CompositeBody(std::vector<boost::shared_ptr<PhysicalBody> > subBodies,
			dWorldID world);

	virtual ~CompositeBody();

	virtual osg::Vec3 getPosition();
	virtual osg::Quat getAttitude();

private:

	void addSubBody(boost::shared_ptr<PhysicalBody> subBody);
	std::vector<boost::shared_ptr<SimpleBody> > flattenSubBodies();

	dMass compositeMass_;
	std::vector<boost::shared_ptr<PhysicalBody> > subBodies_;
};

}

#endif /* COMPOSITEBODY_CPP_ */
