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

Grammar::Grammar(){
    this->rules_.push_back( boost::shared_ptr<Rule>(new Rule(2)));
}

Grammar::Rule::Rule(int iterations){
    this->iterations_ = iterations;
}

boost::shared_ptr<PartRepresentation> buildTree(void){
    // Generate a core component
	boost::shared_ptr<PartRepresentation> corePart = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_CORE_COMPONENT),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());
    return corePart;
}