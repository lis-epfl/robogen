/*
 * PartRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/PartRepresentation.h"
#include <sstream>

#include "evolution/parts/ActiveCardanRepresentation.h"
#include "evolution/parts/ActiveHingeRepresentation.h"
#include "evolution/parts/ActiveWheelRepresentation.h"
#include "evolution/parts/ActiveWhegRepresentation.h"
#include "evolution/parts/CoreComponentRepresentation.h"
#include "evolution/parts/FixedBrickRepresentation.h"
#include "evolution/parts/LightSensorRepresentation.h"
#include "evolution/parts/ParametricBarJointRepresentation.h"
#include "evolution/parts/PassiveCardanRepresentation.h"
#include "evolution/parts/PassiveHingeRepresentation.h"
#include "evolution/parts/PassiveWheelRepresentation.h"
#include "evolution/parts/RotatorRepresentation.h"
#include "evolution/parts/TouchSensorRepresentation.h"

namespace robogen {

PartRepresentationException::PartRepresentationException(const std::string& w) :
														std::runtime_error(w){}

PartRepresentation::PartRepresentation(std::string id,
		int orientation, int arity):
		id_(id), arity_(arity), orientation_(orientation){
	children_.resize(arity_,boost::shared_ptr<PartRepresentation>());
}

PartRepresentation::~PartRepresentation() {
}

std::string &PartRepresentation::getId(){
	return id_;
}

int PartRepresentation::getOrientation(){
	return orientation_;
}

int PartRepresentation::getArity(){
	return arity_;
}

boost::shared_ptr<PartRepresentation> PartRepresentation::getChild(int n){
	if (n<1 || n>arity_){
		std::stringstream ss;
		ss << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with arity " << arity_;
		throw PartRepresentationException(ss.str());
	}
	return children_[n-1];
}

boost::shared_ptr<PartRepresentation> PartRepresentation::setChild(int n,
		boost::shared_ptr<PartRepresentation> part){
	if (n<1 || n>arity_){
		std::stringstream ss;
		ss << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with arity " << arity_;
		throw PartRepresentationException(ss.str());
	}
	part.swap(children_[n-1]);
	return part;
}

boost::shared_ptr<PartRepresentation> PartRepresentation::create(char type,
		std::string id, int orientation, std::vector<double> params){
	switch (type){
	case 'E':
		return boost::shared_ptr<PartRepresentation>(
				new CoreComponentRepresentation(id, orientation));
	case 'F':
		return boost::shared_ptr<PartRepresentation>(
				new FixedBrickRepresentation(id, orientation));
	case 'B':
		if (params.size() != 3){
			std::stringstream ss;
			ss << "Parameter count is not 3 on parametric bar joint with id "
					<< id;
			throw PartRepresentationException(ss.str());
		}
		return boost::shared_ptr<PartRepresentation>(
				new ParametricBarJointRepresentation(id, orientation, params[0],
						params[1], params[2]));
	case 'H':
		return boost::shared_ptr<PartRepresentation>(
				new PassiveHingeRepresentation(id, orientation));
	case 'I':
		return boost::shared_ptr<PartRepresentation>(
				new ActiveHingeRepresentation(id, orientation));
	case 'C':
		return boost::shared_ptr<PartRepresentation>(
				new PassiveCardanRepresentation(id, orientation));
	case 'K':
		return boost::shared_ptr<PartRepresentation>(
				new ActiveCardanRepresentation(id, orientation));
	case 'R':
		return boost::shared_ptr<PartRepresentation>(
				new RotatorRepresentation(id, orientation));
	case 'W':
		if (params.size() != 1){
			std::stringstream ss;
			ss << "Parameter count is not 1 on passive wheel with id " << id;
			throw PartRepresentationException(ss.str());
		}
		return boost::shared_ptr<PartRepresentation>(
				new PassiveWheelRepresentation(id, orientation, params[0]));
	case 'J':
		if (params.size() != 1){
			std::stringstream ss;
			ss << "Parameter count is not 1 on active wheel with id " << id;
			throw PartRepresentationException(ss.str());
		}
		return boost::shared_ptr<PartRepresentation>(
				new ActiveWheelRepresentation(id, orientation, params[0]));
	case 'G':
		if (params.size() != 1){
			std::stringstream ss;
			ss << "Parameter count is not 1 on active wheg with id " << id;
			throw PartRepresentationException(ss.str());
		}
		return boost::shared_ptr<PartRepresentation>(
				new ActiveWhegRepresentation(id, orientation, params[0]));
	case 'L':
		return boost::shared_ptr<PartRepresentation>(
				new LightSensorRepresentation(id, orientation));
	case 'T':
		return boost::shared_ptr<PartRepresentation>(
				new TouchSensorRepresentation(id, orientation));
	default:
		std::stringstream ss;
		ss << "Unknown part type specified: " << type;
		throw PartRepresentationException(ss.str());
	}
}

} /* namespace robogen */
