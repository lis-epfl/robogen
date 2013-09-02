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
#include "evolution/engine/Selector.h"
#include "evolution/engine/Mutator.h"

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
	boost::shared_ptr<Population> pop(new Population(test,8,rng));

	std::cout << "Now creating message" << std::endl;
	robogenMessage::Robot message = pop->getRobot(7)->serialize();

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
	pop->evaluate(sockets);

	std::cout << "Now selecting from evaluated population" << std::endl;
	Selector s(3,rng);
	boost::shared_ptr<Population> pop2(s.select(pop));

	std::cout << "Now mutating new population" << std::endl;
	Mutator m(0.1, 0.1, 0.1, rng);
	m.mutateCrossover(*pop2.get());

	std::cout << "Now evaluating new population" << std::endl;
	pop2->evaluate(sockets);
}


