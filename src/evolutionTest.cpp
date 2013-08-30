/*
 * evolutionTest.cpp
 *
 *  Created on: Aug 29, 2013
 *      Author: lis
 */

#include <iostream>
#include "evolution/RobotRepresentation.h"

using namespace robogen;

int main(){
	RobotRepresentation test("evolutionTest.txt");
	std::cout << "Now creating copy" << std::endl;
	RobotRepresentation copy(test);
}


