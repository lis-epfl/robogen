/*
 * evolutionTest.cpp
 *
 *  Created on: Aug 29, 2013
 *      Author: lis
 */

#include <iostream>
#include <fstream>
#include "robogen.pb.h"
#include "evolution/representation/RobotRepresentation.h"
#include "evolution/engine/Population.h"

using namespace robogen;

int main(){
	RobotRepresentation test("evolutionTest.txt");
	std::cout << "Now creating copy" << std::endl;
	RobotRepresentation copy(test);
	std::cout << "Now assigning" << std::endl;
	RobotRepresentation assign("evolutionTest.txt");
	assign = test;
	std::cout << "Now creating population" << std::endl;
	Population pop(test,100);

	std::cout << "Now creating message" << std::endl;
	robogenMessage::Robot message = pop.getRobot(33)->serialize();
	std::cout << message.DebugString() << std::endl;
	std::cout << "Now writing message" << std::endl;
	std::string fileName("evolutionTest.dat");
	std::ofstream curRobotFile(fileName.c_str(),std::ios::out|std::ios::binary|
			std::ios::trunc);
	message.SerializeToOstream(&curRobotFile);
	curRobotFile.close();

}


