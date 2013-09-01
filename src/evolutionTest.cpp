/*
 * evolutionTest.cpp
 *
 *  Created on: Aug 29, 2013
 *      Author: lis
 */

#include <iostream>
#include <fstream>
#include <boost/random/mersenne_twister.hpp>
#include "robogen.pb.h"
#include "utils/network/TcpSocket.h"
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
	boost::random::mt19937 rng;
	Population pop(test,100,rng);

	std::cout << "Now creating message" << std::endl;
	robogenMessage::Robot message = pop.getRobot(40)->serialize();

	std::cout << "Now writing message" << std::endl;
	std::string fileName("evolutionTest.dat");
	std::ofstream curRobotFile(fileName.c_str(),std::ios::out|std::ios::binary|
			std::ios::trunc);
	message.SerializeToOstream(&curRobotFile);
	curRobotFile.close();

	std::cout << "Now evaluating population" << std::endl;
	std::vector<TcpSocket*> sockets(4);
	for (int i=0; i<4; i++){
		sockets[i] = new TcpSocket;
		if(sockets[i]->open("127.0.0.1", 8001 + i))
			std::cout << "Connection to Simulator Server opened" << std::endl;
		else
			std::cout << "Could not open connection to Simulator Server!" << std::endl;
	}
	pop.evaluate(sockets);

}


