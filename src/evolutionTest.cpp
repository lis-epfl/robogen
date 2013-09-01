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

using namespace robogen;

int main(){
	RobotRepresentation test("evolutionTest.txt");
	std::cout << "Now creating copy" << std::endl;
	RobotRepresentation copy(test);
	std::cout << "Now assigning" << std::endl;
	RobotRepresentation assign("evolutionTest.txt");
	assign = test;
	std::cout << "Now creating message" << std::endl;
	robogenMessage::Robot message = test.serialize();

	std::string fileName("evolutionTest.dat");
	std::ofstream curRobotFile(fileName.c_str(),std::ios::out|std::ios::binary|
			std::ios::trunc);
	message.SerializeToOstream(&curRobotFile);
	curRobotFile.close();
}


