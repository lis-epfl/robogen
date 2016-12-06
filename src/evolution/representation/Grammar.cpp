/*
 * Grammar.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: lis
 */

#include <iostream>
#include <sstream>
#include <math.h>

#include "evolution/representation/Grammar.h"

using namespace robogen;

Grammar::Rule::Rule(int iterations, boost::shared_ptr<SubRobotRepresentation> predecessor,
                        boost::random::mt19937 &rng, float p, float s, int nS){
    this->iterations_ = iterations;
    this->predecessor_ = predecessor;

	nS++;

	this->buildMap_.reset(new effectMap());
	this->deleteMap_.reset(new effectMap());

	boost::shared_ptr<SubRobotRepresentation> successor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*predecessor.get()));

	typedef SubRobotRepresentation::IdPartMap::iterator it_type;
	SubRobotRepresentation::IdPartMap bot = successor->getBody();

	for(it_type iterator = bot.begin(); iterator != bot.end(); iterator++) {
		(*this->deleteMap_)[iterator->first] = "NONE";
	}

	SubRobotRepresentation::IdPartMap parts = successor->getBody();
	boost::random::bernoulli_distribution<double> shouldRemove(p);
	while(shouldRemove(rng) && parts.size()>1){
		//We generate a uniform random distribution for all pieces in the predecessor.
		boost::random::uniform_int_distribution<> dist(0, parts.size() - 1 -1);
		//We get an iterator and we point it to the beginning of the parts.
		SubRobotRepresentation::IdPartMap::const_iterator partToRemove =
				parts.begin();
		//We advance the pointer by a random number of steps, inside its range
		std::advance(partToRemove, dist(rng)+1);

		//We try to remove the part of the subrobot
		bool success = successor->removePart(partToRemove->first, false);

		parts = successor->getBody();
	}

	boost::random::bernoulli_distribution<double> shouldInsert(s);

	while(shouldInsert(rng) && parts.size()<nS){
		//APPEND TIMEEEEEEE
		std::vector<double> parameters;

		boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
			'F',
			"",
			0, //orientation
			parameters);

		SubRobotRepresentation::IdPartMap::const_iterator targetPart = predecessor->getBody().begin();

		std::advance(targetPart,1);

		successor->insertPart(targetPart->first,
				0, //Parent slot
				newPart,
				0, //newPartSlot
				0 ? NeuronRepresentation::OSCILLATOR :
				NeuronRepresentation::SIGMOID,
				false);

		buildStep tmpStep;

		tmpStep.parentPartId = targetPart->first;
		tmpStep.parentPartSlot = 0;
		tmpStep.newPart = newPart;
		tmpStep.newPartSlot = 0;
		tmpStep.motorNeuronType = NeuronRepresentation::SIGMOID;

		this->insertions_.push_back(tmpStep);
		parts = successor->getBody();
	}

	bot = successor->getBody();

	for(it_type iterator = bot.begin(); iterator != bot.end(); iterator++) {
		(*this->buildMap_)[iterator->first] = "NONE";
	}

	this->successor_=successor;
}

int Grammar::Rule::getNumIterations(){
	return this->iterations_;
}

bool areTreesSimilar(boost::shared_ptr<PartRepresentation> model, boost::shared_ptr<PartRepresentation> target, boost::shared_ptr<Grammar::Rule::effectMap> theMap){
	//Comparison part
	//Are types unequeal
	if(model->getType()!=target->getType()){
		return false;
	} else {
		(*theMap)[model->getId()] = target->getId();
		//Does the model have children?
		if(model->getChildrenCount()>0){
			//Then, does the target have also?
			if(target->getChildrenCount()>0){
				//Now we'll check if the target share's the model's children
				for(int i=0;i < model->getChildrenCount(); i++){
					//Positions must match
					int pos = model->getChild(i)->getPosition();
					//We check against all children at the target
					bool kidsMatch=false;
					for(int j=0; j<target->getChildrenCount(); j++){
						if(pos == target->getChild(j)->getPosition()){
							//if the children do not match (recursively)
							if(!areTreesSimilar(model->getChild(i), target->getChild(j), theMap)){
								return false;
							}
							kidsMatch=true;
							break;
						}
					}
					//If we didn't find a child in the same slot
					if(!kidsMatch){
						return false;
					}
				}
				return true;
			} else {
				return false;
			}
		} else {
			return true;
		}
	}
}

bool Grammar::Rule::matchesPredecessor(boost::shared_ptr<PartRepresentation> candidate){
	typedef SubRobotRepresentation::IdPartMap::iterator it_type;

	boost::shared_ptr<PartRepresentation> tree = this->predecessor_->getTree();

	bool match = areTreesSimilar(tree->getChild(0), candidate, this->deleteMap_);

	return match;
}

bool Grammar::Rule::applyRule(boost::shared_ptr<SubRobotRepresentation> robot, boost::shared_ptr<PartRepresentation> node){
	boost::shared_ptr<SubRobotRepresentation> successor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*this->predecessor_.get()));

	bool match = areTreesSimilar(this->predecessor_->getTree()->getChild(0), node, this->buildMap_);

	typedef Grammar::Rule::effectMap::iterator it_type;
	//std::cout << "First I was like:\n"; 
	for(it_type iterator2 = this->buildMap_->begin(); iterator2 != this->buildMap_->end(); iterator2++) {
		//std::cout << iterator2->first << " and " << iterator2->second << std::endl;
	}

	for(int i=0; i< this->insertions_.size(); i++){
		buildStep tmpStep = this->insertions_.at(i);

		successor->insertPart(tmpStep.parentPartId,
				tmpStep.parentPartSlot,
				tmpStep.newPart,
				tmpStep.newPartSlot,
				tmpStep.motorNeuronType = NeuronRepresentation::SIGMOID,
				false);

		robot->insertPart((*this->buildMap_)[tmpStep.parentPartId],
				tmpStep.parentPartSlot,
				boost::shared_ptr<PartRepresentation>(new PartRepresentation(*tmpStep.newPart.get())),
				tmpStep.newPartSlot,
				tmpStep.motorNeuronType = NeuronRepresentation::SIGMOID,
				false);

		//std::cout << "Ready: " << robot->getNodeById(tmpStep.parentPartId)->getId() << std::endl;//getChild(tmpStep.newPartSlot)->getId() << std::endl;
		//*(this->buildMap_)[tmpStep.parentPartId] = robot->getBody()[(*this->buildMap_)[tmpStep.parentPartId]].lock()->getChild(tmpStep.parentPartSlot)->getId();
		(*this->buildMap_)[successor->getNodeById(tmpStep.parentPartId)->getChild(tmpStep.parentPartSlot)->getId()] = robot->getNodeById(tmpStep.parentPartId)->getId();
	}
}

boost::shared_ptr<SubRobotRepresentation> Grammar::Rule::getSuccessor(void){
	return this->successor_;
}

boost::shared_ptr<SubRobotRepresentation> Grammar::Rule::getPredecessor(void){
	return this->predecessor_;
}

int Grammar::getNumberOfRules(){
	return this->rules_.size();
}

boost::shared_ptr<Grammar::Rule> Grammar::getRule(int id){
	return this->rules_.at(id);
}

Grammar::Grammar(boost::shared_ptr<SubRobotRepresentation> axiom){
	//Make the pointer point to a new empty subrobot representation
    this->axiom_.reset(new SubRobotRepresentation());
	//Deep copy the passed axiom.
	*this->axiom_ = *axiom;

    //Hardcoded Rule:
	//We MUTATE THE GRAMMAR AAAAAAHHHHHH
	boost::shared_ptr<SubRobotRepresentation> predecessor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation());
	//We give a core to the predecessor, at least, to not be mean.
	predecessor->init();

	std::vector<double> parameters;

	boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
		'F',
		"",
		0, //orientation
		parameters);

	SubRobotRepresentation::IdPartMap::const_iterator targetPart = predecessor->getBody().begin();

	predecessor->insertPart(targetPart->first,
			0, //Parent slot
			newPart,
			0, //newPartSlot
			0 ? NeuronRepresentation::OSCILLATOR :
			NeuronRepresentation::SIGMOID,
			false);
}

boost::shared_ptr<SubRobotRepresentation> Grammar::getAxiom(void){
	return this->axiom_;
}

boost::shared_ptr<SubRobotRepresentation> Grammar::buildTree(void){

	boost::shared_ptr<SubRobotRepresentation> final;

	final.reset(new SubRobotRepresentation());
	//Deep copy of values only
	*final = *this->axiom_;

	typedef SubRobotRepresentation::IdPartMap::iterator it_type;

	SubRobotRepresentation::IdPartMap bot = final->getBody();

	int nRules = this->rules_.size();

	std::cout << "FIRST: " << final->getBody().size() << std::endl;

	//Iterate over every rule
	for(int r=0;r<nRules;r++){

		//Repeat each rule as many times as required
		for(int n=this->rules_.at(r)->getNumIterations();n>0;n--){

			//look for the rule through all the tree
			for(it_type iterator = bot.begin(); iterator != bot.end(); iterator++) {

				if(this->rules_.at(r)->matchesPredecessor(iterator->second.lock())){

					this->rules_.at(r)->applyRule(final, iterator->second.lock());

				}
			}

			bot=final->getBody();
		}
	}

	std::cout << "LAST: " << final->getBody().size() << std::endl;

	return final;
}

bool Grammar::addRule(boost::shared_ptr<Grammar::Rule> newRule){
	this->rules_.push_back(newRule);
	return true;
}