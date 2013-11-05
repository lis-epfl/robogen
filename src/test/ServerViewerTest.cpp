/*
 * @(#) ServerViewer.cpp   1.0   Mar 6, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include "utils/network/TcpSocket.h"
#include "utils/network/ProtobufPacket.h"
#include "utils/RobogenUtils.h"

#include "Robot.h"
#include "Robogen.h"

using namespace robogen;

/**
 * Used to test ServerViewer.cpp and Server.cpp
 * Reads a ROBOGEN robot from the specified file, connect to the server at the specified port
 * and send the robot description to it
 */
int main(int argc, char *argv[]) {

	if (argc != 4) {
		std::cout
				<< "Please provide the server ip and port and a file containing the robot description as input '"
				<< std::string(argv[0]) << "SERVER_IP SERVER_PORT robot.dat'"
				<< std::endl;
		return EXIT_FAILURE;
	}

	std::string serverIp(argv[1]);
	int serverPort = std::atoi(argv[2]);
	std::string fileName(argv[3]);

	// ---------------------------------------
	// Robot decoding
	// ---------------------------------------
	std::ifstream robotFile(fileName.c_str());
	if (!robotFile.is_open()) {
		std::cout << "Cannot open " << std::string(argv[1]) << ". Quit."
				<< std::endl;
		return EXIT_FAILURE;
	}

	ProtobufPacket<robogenMessage::Robot> robogenPacket;

	robotFile.seekg(0, robotFile.end);
	unsigned int packetSize = robotFile.tellg();
	robotFile.seekg(0, robotFile.beg);

	std::vector<unsigned char> packetBuffer;
	packetBuffer.resize(packetSize);
	robotFile.read((char*) &packetBuffer[0], packetSize);
	robogenPacket.decodePayload(packetBuffer);

	// ---------------------------------------
	// Connect to server and send robot description
	// ---------------------------------------

	while (true) {

		std::cout
				<< "Press s to send another time the robot, another key to exit."
				<< std::endl;
		char c;
		std::cin >> c;
		if (c == 's') {

			std::cout << "Sending robot" << std::endl;

			TcpSocket socket;
			if (!socket.open(serverIp, serverPort)) {
				std::cout << "Cannot connect to server @ " << serverIp << ":"
						<< serverPort << std::endl;
				return EXIT_FAILURE;
			}

			std::vector<unsigned char> buffer;
			robogenPacket.forge(buffer);
			socket.write(buffer);

			std::cout << "Robot sent" << std::endl;

			socket.close();

		} else {
			break;
		}
	}

	return EXIT_SUCCESS;
}
