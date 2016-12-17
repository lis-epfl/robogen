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
                        boost::random::mt19937 &rng, boost::shared_ptr<EvolverConfiguration> conf){
	//We store the number of iterations for this rule
    this->iterations_ = iterations;

	//we also store the predecessor, we keep the shared pointer
    this->predecessor_ = predecessor;

	//We start two new maps, one to remove pieces from the predecessor
	this->deleteMap_.reset(new effectMap());

	//one to add pieces until we arrive to the successor.
	this->buildMap_.reset(new effectMap());

	// we declare a new SubRobotRep that will eventually be the successor.
	boost::shared_ptr<SubRobotRepresentation> successor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*predecessor.get()));


	typedef SubRobotRepresentation::IdPartMap::iterator it_type;
	SubRobotRepresentation::IdPartMap bot = successor->getBody();

	//We empty completely the delete map in the second entrances
	for(it_type iterator = bot.begin(); iterator != bot.end(); iterator++) {
		(*this->deleteMap_)[iterator->first] = "NONE";
	}

	SubRobotRepresentation::IdPartMap parts = successor->getBody();

	//Probability of removing a piece
	boost::random::bernoulli_distribution<double> shouldRemove(conf->deletionProb);
	//probability of adding a piece
	boost::random::bernoulli_distribution<double> shouldInsert(conf->insertionProb);

	//We loop until there is nothing to remove or we fail to query a remove
	while(shouldRemove(rng) && parts.size()>1){
		//We generate a uniform random distribution for all pieces in the predecessor.
		boost::random::uniform_int_distribution<> dist(0, parts.size() - 1 -1);
		//We get an iterator and we point it to the beginning of the parts.
		SubRobotRepresentation::IdPartMap::const_iterator partToRemove =
				parts.begin();
		//We advance the pointer by a random number of steps, skipping the core
		std::advance(partToRemove, dist(rng)+1);

		//We try to remove the part of the subrobot
		bool success = successor->removePart(partToRemove->first, false);

		//we update the bodyMap
		parts = successor->getBody();
	}

	//Now to the insertion block

	while(shouldInsert(rng) && parts.size()<conf->maxSuccessorPieces+1){
		//APPEND TIMEEEEEEE

		int offSet = 0;

		if(successor->getBody().size()>1){
			offSet=1;
		}

		boost::random::uniform_int_distribution<> dist(offSet, successor->getBody().size() - 1);

		SubRobotRepresentation::IdPartMap::const_iterator parent;


		boost::shared_ptr<PartRepresentation> parentPart;
		// find a parent with arity > 0 (will exist, since at the very least will
		// be the core)

		do {
			parent = successor->getBody().begin();
			std::advance(parent, dist(rng));
			parentPart = parent->second.lock();
		} while (parentPart->getArity() == 0);

		boost::random::uniform_int_distribution<> distType(0,
			conf->allowedBodyPartTypes.size() - 1);
		char type = conf->allowedBodyPartTypes[distType(rng)];

		// Randomly generate node orientation
		boost::random::uniform_int_distribution<> orientationDist(0, 3);
		unsigned int curOrientation = orientationDist(rng);

		// Randomly generate parameters
		unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type));
		std::vector<double> parameters;
		boost::random::uniform_01<double> paramDist;
		for (unsigned int i = 0; i < nParams; ++i) {
			parameters.push_back(paramDist(rng));
		}

		// Create the new part
		boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
				type, "", curOrientation, parameters);

		//Create a backup Part, before it got inserted in the predecessor
		boost::shared_ptr<PartRepresentation> backupNewPart = PartRepresentation::create(
				type, "", curOrientation, parameters);

		unsigned int newPartSlot = 0;

		if (newPart->getArity() > 0) {
			// Generate a random slot in the new node, if it has arity > 0
			boost::random::uniform_int_distribution<> distNewPartSlot(0,
					newPart->getArity() - 1);
			newPartSlot = distNewPartSlot(rng);
		}
		// otherwise just keep it at 0... inserting part will fail if arity is 0 and
		// there were previously parts attached to the parent's chosen slot

		boost::random::bernoulli_distribution<double> oscillatorNeuronDist_(conf->pOscillatorNeuron);

		int mNType = oscillatorNeuronDist_(rng) ? NeuronRepresentation::OSCILLATOR :
						NeuronRepresentation::SIGMOID;

		// Sample a random slot
		boost::random::uniform_int_distribution<> slotDist(0,
													parentPart->getArity() - 1);
		unsigned int parentSlot = slotDist(rng);

		if(successor->getBody().size()==1){
			parentSlot=0;
		}

		bool succeed = successor->insertPart(parent->first, parentSlot, newPart, newPartSlot,
				mNType, false);

		//If the insertion was successfull, we keep this step in the insertions.
		if(succeed){
			buildStep tmpStep;

			tmpStep.parentPartId = parent->first;
			tmpStep.parentPartSlot = parentSlot;
			tmpStep.newPart = backupNewPart;
			tmpStep.newPartSlot = newPartSlot;
			tmpStep.motorNeuronType = mNType;

			this->insertions_.push_back(tmpStep);
		}

		//we update the body map
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

bool generateCloneMap(boost::shared_ptr<PartRepresentation> model, boost::shared_ptr<PartRepresentation> target, boost::shared_ptr<Grammar::Rule::effectMap> theMap){
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
							if(!generateCloneMap(model->getChild(i), target->getChild(j), theMap)){
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

bool matchingTree(boost::shared_ptr<PartRepresentation> original, boost::shared_ptr<PartRepresentation> wannabe){
	//Comparison part
	//different types, immediate false
	if(wannabe->getType()!=original->getType()){
		return false;
	} else {
		//Does the model have children?
		if(original->getChildrenCount()>0){
			//Then, does the target have also?
			if(wannabe->getChildrenCount()>0){
				//Now we'll check if the target share's the model's children
				for(int i=0;i < original->getChildrenCount(); i++){
					//Positions must match
					int pos = original->getChild(i)->getPosition();
					//We check against all children at the target
					bool kidsMatch=false;
					for(int j=0; j<wannabe->getChildrenCount(); j++){
						if(pos == wannabe->getChild(j)->getPosition()){
							//There is a child in the same slot... is it the same?
							if(!matchingTree(original->getChild(i), wannabe->getChild(j))){
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
				//all kids matched, so it is true
				return true;
			} else {
				//immediate false, no children!
				return false;
			}
		} else {
			//They match in type, it doesn't matter if the wannabe has children
			return true;
		}
	}
}

bool Grammar::Rule::matchesPredecessor(boost::shared_ptr<PartRepresentation> candidate){
	boost::shared_ptr<PartRepresentation> tree = this->predecessor_->getTree();
	return matchingTree(tree->getChild(0), candidate);
}

bool Grammar::Rule::applyRule(boost::shared_ptr<SubRobotRepresentation> robot, boost::shared_ptr<PartRepresentation> node){
	
	boost::shared_ptr<SubRobotRepresentation> successor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*this->predecessor_.get()));

	boost::shared_ptr<Rule::effectMap> tmpMap = boost::shared_ptr<Rule::effectMap>(new Rule::effectMap(*this->buildMap_.get()));

	bool match = generateCloneMap(this->predecessor_->getTree()->getChild(0), node, tmpMap);

	for(int i=0; i< this->insertions_.size(); i++){
		buildStep tmpStep = this->insertions_.at(i);

		boost::shared_ptr<PartRepresentation> newPart = boost::shared_ptr<PartRepresentation>(new PartRepresentation(*tmpStep.newPart.get()));

		successor->insertPart(tmpStep.parentPartId,
				tmpStep.parentPartSlot,
				newPart,
				tmpStep.newPartSlot,
				tmpStep.motorNeuronType,
				false);

		boost::shared_ptr<PartRepresentation> newPart2 = boost::shared_ptr<PartRepresentation>(new PartRepresentation(*tmpStep.newPart.get()));

		bool success = robot->insertPart((*tmpMap)[tmpStep.parentPartId],
				tmpStep.parentPartSlot,
				newPart2,
				tmpStep.newPartSlot,
				tmpStep.motorNeuronType,
				false);

		if(!success){
			return false;
		}

		(*tmpMap)[successor->getNodeById(tmpStep.parentPartId)->getChild(tmpStep.parentPartSlot)->getId()] = newPart2->getId();
	}

	return true;
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

void Grammar::popLastRule(void){
	if(this->rules_.size()>0){
		this->rules_.pop_back();
	}
}

Grammar::Grammar(boost::shared_ptr<SubRobotRepresentation> axiom){
	//Make the pointer point to a new empty subrobot representation
    this->axiom_.reset(new SubRobotRepresentation());
	//Deep copy the passed axiom.
	*this->axiom_ = *axiom;

	this->lastBuildWorked=true;

    //Hardcoded Rule:
	//We MUTATE THE GRAMMAR AAAAAAHHHHHH
	boost::shared_ptr<SubRobotRepresentation> predecessor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation());
	//We give a core to the predecessor, at least, to not be mean.
	predecessor->init();

	std::vector<double> parameters;

	//Is a default value of 4 faces, Idealy the parameter must be generate randomly
	parameters.push_back(4);
	boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
		'P',
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

	//emtpy pointer for the final build
	boost::shared_ptr<SubRobotRepresentation> final;

	final.reset(new SubRobotRepresentation());
	//We start with the axiom
	*final = *this->axiom_;

	typedef SubRobotRepresentation::IdPartMap::iterator it_type;
	SubRobotRepresentation::IdPartMap bot = final->getBody();

	int nRules = this->rules_.size();

	//Iterate over every rule
	for(int r=0;r<nRules;r++){

		//Repeat each rule as many times as required
		for(int n=this->rules_.at(r)->getNumIterations();n>0;n--){

			//look for the rule through all the tree
			for(it_type iterator = bot.begin(); iterator != bot.end(); iterator++) {

				if(this->rules_.at(r)->matchesPredecessor(iterator->second.lock())){

					if(!this->rules_.at(r)->applyRule(final, iterator->second.lock())){
						//The robot can't be built!
						*final = *this->axiom_;
						this->lastBuildWorked=false;
						return final;
					}

				}
			}

			bot=final->getBody();
		}
	}
	
	this->lastBuildWorked=true;
	return final;
}

bool Grammar::addRule(boost::shared_ptr<Grammar::Rule> newRule){
	this->rules_.push_back(newRule);
	return true;
}

bool Grammar::lastBuildFailed(){
	return !this->lastBuildWorked;
}