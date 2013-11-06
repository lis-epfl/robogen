/*
 * PartRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include <iostream>
#include <sstream>

#include "evolution/representation/PartRepresentation.h"
#include "PartList.h"

namespace robogen {

PartRepresentation::PartRepresentation(std::string id, int orientation,
		int arity, const std::string& type, const std::vector<double>& params,
		const std::vector<std::string>& motors,
		const std::vector<std::string>& sensors) :
		id_(id), orientation_(orientation), arity_(arity), type_(type), params_(
				params), motors_(motors), sensors_(sensors) {

	children_.resize(arity_, boost::shared_ptr<PartRepresentation>());

}

PartRepresentation::~PartRepresentation() {

}

std::string &PartRepresentation::getId() {
	return id_;
}

int PartRepresentation::getOrientation() {

	return orientation_;
}

void PartRepresentation::setId(std::string newid) {
	id_ = newid;
}

int PartRepresentation::getArity() {
	return arity_;
}

int PartRepresentation::numDescendants() {

	int descendants = 0;
	for (unsigned int i = 0; i < children_.size(); ++i) {
		// child and all its children
		if (children_[i].get()) {
			++descendants += children_[i]->numDescendants();
		}
	}
	return descendants;

}

unsigned int PartRepresentation::getChildrenCount() {

	int count = 0;
	for (unsigned int i = 0; i < children_.size(); ++i) {
		if (children_[i].get()) {
			count++;
		}
	}
	return count;

}

std::vector<unsigned int> PartRepresentation::getFreeSlots() {

	std::vector<unsigned int> freeSlots;
	for (unsigned int i = 0; i < children_.size(); ++i) {
		if (!children_[i].get()) {
			freeSlots.push_back(i);
		}
	}
	return freeSlots;

}

std::string &PartRepresentation::getType() {
	return type_;
}

boost::shared_ptr<PartRepresentation> PartRepresentation::getChild(int n) {
	if (n < 0 || n > arity_ - 1) {
		std::cout << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with arity " << arity_ << std::endl;
		return boost::shared_ptr<PartRepresentation>();
	}
	return children_[n];
}

bool PartRepresentation::setChild(int n,
		boost::shared_ptr<PartRepresentation> part) {

	if (n < 0 || n > arity_ - 1) {
		std::cout << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with arity " << arity_ << std::endl;
		return false;
	}
	// don't try to access part if void
	if (part) {
		part->setParent(this);
		part->setPosition(n);
	}
	children_[n] = part;
	return true;

}

boost::shared_ptr<PartRepresentation> PartRepresentation::create(char type,
		std::string id, int orientation, std::vector<double> params) {

	if (PART_TYPE_MAP.count(type) == 0) {
		std::cout << "Unknown part type '" << type << "'" << std::endl;
		return boost::shared_ptr<PartRepresentation>();
	}

	std::string partType = PART_TYPE_MAP[type];
	if (params.size() != PART_TYPE_PARAM_COUNT_MAP[partType]) {
		std::cout << "The parameter count (" << params.size()
				<< ") does not equal the requested parameter count ("
				<< PART_TYPE_PARAM_COUNT_MAP[partType] << ") for the part: '"
				<< id << "'" << std::endl;
		return boost::shared_ptr<PartRepresentation>();
	}

	return boost::shared_ptr<PartRepresentation>(
			new PartRepresentation(id, orientation,
					PART_TYPE_ARITY_MAP[partType], partType, params,
					PART_TYPE_MOTORS_MAP[partType],
					PART_TYPE_SENSORS_MAP[partType]));

}

void PartRepresentation::addSubtreeToBodyMessage(
		robogenMessage::Body *bodyMessage, bool amIRoot) {

	// first, insert self
	robogenMessage::BodyPart* serialization = bodyMessage->add_part();

	// required string id = 1;
	serialization->set_id(id_);

	// required string type = 2;
	serialization->set_type(this->getType());

	// required bool root = 3;
	serialization->set_root(amIRoot);

	// repeated EvolvableParameter evolvableParam = 4;
	for (unsigned int i = 0; i < params_.size(); ++i) {
		robogenMessage::EvolvableParameter *param =
				serialization->add_evolvableparam();
		param->set_paramvalue(params_[i]);
	}

	// required int32 orientation = 5;
	serialization->set_orientation(orientation_);

	// treat children, including connection
	for (int i = 0; i < arity_; i++) {
		if (this->getChild(i)) {
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

std::vector<std::string> PartRepresentation::getAncestorsIds() {

	std::vector<std::string> ids;
	if (parent_) {
		ids.push_back(parent_->getId());
		std::vector<std::string> tmp = parent_->getAncestorsIds();
		ids.insert(ids.end(), tmp.begin(), tmp.end());
	}
	return ids;

}

std::vector<std::string> PartRepresentation::getDescendantsIds() {

	std::vector<std::string> ids;
	for (unsigned int i = 0; i < children_.size(); ++i) {
		// child and all its children
		if (children_[i].get()) {

			// Add children ID
			ids.push_back(children_[i]->getId());

			// Add all the descendants ids
			std::vector<std::string> tmp = children_[i]->getDescendantsIds();
			ids.insert(ids.end(), tmp.begin(), tmp.end());
		}
	}
	return ids;

}

boost::shared_ptr<PartRepresentation> PartRepresentation::cloneSubtree() {

	boost::shared_ptr<PartRepresentation> theClone(
			new PartRepresentation(this->getId(), this->getOrientation(),
					this->getArity(), this->getType(), this->getParams(),
					this->getMotors(), this->getSensors()));
	// deep copy all children
	for (int i = 0; i < this->getArity(); i++) {
		if (this->getChild(i)) {
			theClone->setChild(i, this->getChild(i)->cloneSubtree());
		}
	}
	return theClone;

}

std::vector<double> PartRepresentation::getParams() {
	return params_;
}

void PartRepresentation::setParent(PartRepresentation* parent) {
	parent_ = parent;
}

PartRepresentation* PartRepresentation::getParent() {
	return parent_;
}

void PartRepresentation::setPosition(int position) {
	position_ = position;
}

int PartRepresentation::getPosition() {
	return position_;
}

std::vector<std::string> PartRepresentation::getMotors() {
	return motors_;
}

std::vector<std::string> PartRepresentation::getSensors() {
	return sensors_;
}

} /* namespace robogen */
