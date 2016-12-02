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

Grammar::Rule::Rule(int iterations, boost::shared_ptr<PartRepresentation> predecessor,
                        boost::shared_ptr<PartRepresentation> successor){
    this->iterations_ = iterations;
    this->predecessor_ = predecessor;
    this->successor_ = successor;

	this->ignoreChildrenCount_=true;
	this->ignorePosition_=true;
	this->ignoreOrientation_=true;
}

int Grammar::Rule::getNumIterations(){
	return this->iterations_;
}

void Grammar::Rule::ignoreChildrenCount(bool state){
	this->ignoreChildrenCount_=state;
}

void Grammar::Rule::ignorePosition(bool state){
	this->ignorePosition_=state;
}

void Grammar::Rule::ignoreOrientation(bool state){
	this->ignoreOrientation_=state;
}

bool Grammar::Rule::matchesPredecessor(boost::shared_ptr<PartRepresentation> candidate){
	bool matches=false;
	if(candidate->getType()==this->predecessor_->getType())
		if(candidate->getChildrenCount()==this->predecessor_->getChildrenCount() || this->ignoreChildrenCount_)
			matches=true;
	return matches;
}

Grammar::Grammar(boost::shared_ptr<SubRobotRepresentation> r){

    this->axiom_.reset(new SubRobotRepresentation());
	//Deep copy
	*this->axiom_ = *r;

    //Hardcoded Rule:
    boost::shared_ptr<PartRepresentation> searchPattern = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_FIXED_BRICK),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());

    boost::shared_ptr<PartRepresentation> replacePattern = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_FIXED_BRICK),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());
    this->rules_.push_back( boost::shared_ptr<Rule>(new Rule(1, searchPattern, replacePattern)));
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
				//if(iterator->second.lock()->getType() == PART_TYPE_FIXED_BRICK){

					if(iterator->second.lock()->getChildrenCount()==0 ){

						char type = 'F';

						unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type));
						std::vector<double> parameters;
						for (unsigned int i = 0; i < nParams; ++i) {
							parameters.push_back(0.1f);
						}

						boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
							type,
							"",
							0, //orientation
							parameters);

						final->insertPart(iterator->first,
							0, //Parent slot
							newPart,
							0, //newPartSlot
							0 ? NeuronRepresentation::OSCILLATOR :
									NeuronRepresentation::SIGMOID,
									false);
					}

				}
			}

			bot=final->getBody();
		}
	}

	std::cout << "LAST: " << final->getBody().size() << std::endl;

	return final;
}