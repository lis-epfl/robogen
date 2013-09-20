/*
 * PartRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include "evolution/representation/PartRepresentation.h"
#include <sstream>
#include <iostream>

#include "evolution/representation/parts/ActiveCardanRepresentation.h"
#include "evolution/representation/parts/ActiveHingeRepresentation.h"
#include "evolution/representation/parts/ActiveWheelRepresentation.h"
#include "evolution/representation/parts/ActiveWhegRepresentation.h"
#include "evolution/representation/parts/CoreComponentRepresentation.h"
#include "evolution/representation/parts/FixedBrickRepresentation.h"
#include "evolution/representation/parts/LightSensorRepresentation.h"
#include "evolution/representation/parts/ParametricBarJointRepresentation.h"
#include "evolution/representation/parts/PassiveCardanRepresentation.h"
#include "evolution/representation/parts/PassiveHingeRepresentation.h"
#include "evolution/representation/parts/PassiveWheelRepresentation.h"
#include "evolution/representation/parts/RotatorRepresentation.h"
#include "evolution/representation/parts/TouchSensorRepresentation.h"

namespace robogen {

PartRepresentation::PartRepresentation(std::string id,
		int orientation, int arity, std::string type):
		id_(id), arity_(arity), orientation_(orientation), type_(type),
		parent_(NULL) {
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

int PartRepresentation::numDescendants(){
	int descendants = 0;
	for (unsigned int i=0; i<children_.size(); ++i){
		// child and all its children
		if (children_[i].get()){
			++descendants += children_[i]->numDescendants();
		}
	}
	return descendants;
}

std::string &PartRepresentation::getType(){
	return type_;
}

boost::shared_ptr<PartRepresentation> PartRepresentation::getChild(int n){
	if (n<1 || n>arity_){
		std::cout << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with arity " << arity_ << std::endl;
		return boost::shared_ptr<PartRepresentation>();
	}
	return children_[n-1];
}

bool PartRepresentation::setChild(int n,
		boost::shared_ptr<PartRepresentation> part){
	if (n<1 || n>arity_){
		std::cout << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with arity " << arity_ << std::endl;
		return false;
	}
	// don't try to access part if void
	if (part){
		part->setParent(this);
		part->setPosition(n);
	}
	children_[n-1] = part;
	return true;
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
			std::cout << "Parameter count is not 3 on parametric bar joint "\
					"with id " << id << std::endl;
			return boost::shared_ptr<PartRepresentation>();
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
			std::cout << "Parameter count is not 1 on passive wheel with id " <<
					id << std::endl;
			return boost::shared_ptr<PartRepresentation>();
		}
		return boost::shared_ptr<PartRepresentation>(
				new PassiveWheelRepresentation(id, orientation, params[0]));
	case 'J':
		if (params.size() != 1){
			std::cout << "Parameter count is not 1 on active wheel with id " <<
					id << std::endl;
			return boost::shared_ptr<PartRepresentation>();
		}
		return boost::shared_ptr<PartRepresentation>(
				new ActiveWheelRepresentation(id, orientation, params[0]));
	case 'G':
		if (params.size() != 1){
			std::cout << "Parameter count is not 1 on active wheg with id " <<
					id;
			return boost::shared_ptr<PartRepresentation>();
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
		std::cout << "Unknown part type specified: " << type << std::endl;
		return boost::shared_ptr<PartRepresentation>();
	}
}

void PartRepresentation::addSubtreeToBodyMessage(
		robogenMessage::Body *bodyMessage, bool amIRoot){
	// first, insert self
	robogenMessage::BodyPart* serialization = bodyMessage->add_part();
	// required string id = 1;
	serialization->set_id(id_);
	// required string type = 2;
	serialization->set_type(this->getType());
	// required bool root = 3;
	serialization->set_root(amIRoot);
	// repeated EvolvableParameter evolvableParam = 4;
	for (std::map<std::string, double>::iterator it = params_.begin();
			it != params_.end(); it++){
		robogenMessage::EvolvableParameter *param =
				serialization->add_evolvableparam();
		param->set_paramname(it->first);
		param->set_paramvalue(it->second);
	}
	// required int32 orientation = 5;
	serialization->set_orientation(orientation_);

	// treat children, including connection
	for (int i=1; i<=arity_; i++){
		if (this->getChild(i)){
			robogenMessage::BodyConnection *connection =
					bodyMessage->add_connection();
			connection->set_src(id_);
			connection->set_srcslot(i);
			connection->set_dest(this->getChild(i)->getId());
			connection->set_destslot(0);
			this->getChild(i)->addSubtreeToBodyMessage(bodyMessage, false);
		}
	}
}

void PartRepresentation::setParent(PartRepresentation *parent){
	parent_ = parent;
}

PartRepresentation *PartRepresentation::getParent(){
	return parent_;
}

void PartRepresentation::setPosition(int position){
	position_ = position;
}

int PartRepresentation::getPosition(){
	return position_;
}

} /* namespace robogen */
