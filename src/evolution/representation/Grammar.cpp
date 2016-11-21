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

Grammar::Grammar(boost::shared_ptr<PartRepresentation> axiom){

    this->axiom_ = axiom;

    //Hardcoded Rule:
    boost::shared_ptr<PartRepresentation> searchPattern = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_FIXED_BRICK),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());

    boost::shared_ptr<PartRepresentation> replacePattern = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_FIXED_BRICK),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());
    this->rules_.push_back( boost::shared_ptr<Rule>(new Rule(2, searchPattern, replacePattern)));
}

Grammar::Rule::Rule(int iterations, boost::shared_ptr<PartRepresentation> predecessor,
                        boost::shared_ptr<PartRepresentation> successor){
    this->iterations_ = iterations;
    this->predecessor_ = predecessor;
    this->successor_ = successor;
}

boost::shared_ptr<PartRepresentation> buildTree(void){
    // Generate a core component
	boost::shared_ptr<PartRepresentation> corePart = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_CORE_COMPONENT),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());
    return corePart;
}