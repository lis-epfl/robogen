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
	boost::random::mt19937 rng;
	boost::shared_ptr<Population> pop(new Population(test,48,rng));
	boost::shared_ptr<Population> current(pop), next;

	/*std::cout << "Now writing message" << std::endl;
	std::string fileName("evolutionTest.dat");
	std::ofstream curRobotFile(fileName.c_str(),std::ios::out|std::ios::binary|
			std::ios::trunc);
	message.SerializeToOstream(&curRobotFile);
	curRobotFile.close();*/

	std::vector<TcpSocket*> sockets(4);
	for (int i=0; i<4; i++){
		sockets[i] = new TcpSocket;
		if(sockets[i]->open("127.0.0.1", 8001 + i))
			std::cout << "Connection to Simulator Server opened" << std::endl;
		else
			std::cout << "Could not open connection to Simulator Server!" << std::endl;
	}

	pop->evaluate(sockets);

	Selector s(3,rng);
	Mutator m(0.05, 0.7, 0.1, rng);
	for (int i=0; i<10; i++){
		next = s.select(current);
		current = next;
		m.mutateCrossover(*current.get());
		current->evaluate(sockets);
	}
}


