/*
 * @(#) RobogenServer.cpp   1.0   Mar 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc (guillaume.leclerc@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#include <iostream>
#include <chrono>
#include "socket.io-client-cpp/src/sio_client.h"
#include "socket.io-client-cpp/src/sio_message.h"
#include "socket.io-client-cpp/src/sio_socket.h"

#include "config/ConfigurationReader.h"
#include "config/RobogenConfig.h"
#include "scenario/Scenario.h"
#include "scenario/ScenarioFactory.h"
#include "utils/network/ProtobufPacket.h"
#include "utils/network/TcpSocket.h"
#include "utils/RobogenCollision.h"
#include "utils/RobogenUtils.h"
#include "Models.h"
#include "RenderModels.h"
#include "Robogen.h"
#include "Robot.h"
#include "robogen.pb.h"

#include "viewer/Viewer.h"
#include "Simulator.h"

using namespace robogen;

// ODE World
dWorldID odeWorld;

// Container for collisions
dJointGroupID odeContactGroup;

bool interrupted;

#include <functional>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>


using namespace sio;
using namespace std;
std::mutex _lock;

std::condition_variable_any _cond;
bool connect_finish = false;

class connection_listener
{
	sio::client &handler;

	public:

	connection_listener(sio::client& h):
		handler(h)
	{
	}


	void on_connected()
	{
		_lock.lock();
		_cond.notify_all();
		connect_finish = true;
		_lock.unlock();
	}
	void on_close(client::close_reason const& reason)
	{
		std::cout<<"sio closed "<<std::endl;
		exit(0);
	}

	void on_fail()
	{
		std::cout<<"sio failed "<<std::endl;
		exit(0);
	}
};

int participants = -1;

boost::random::mt19937 rng;
socket::ptr current_socket;

void bind_events(socket::ptr &socket)
{
	current_socket->on("requestTask", sio::socket::event_listener_aux([&](string const& name, message::ptr const& data, bool isAck,message::list &ack_resp)
				{
				_lock.lock();
				const std::string id = data->get_map()["id"]->get_string();
				std::cout << "I have a new task (id:" << id << ")" << std::endl;
				std::vector<message::ptr> content = data->get_map()["content"]->get_map()["packet"]->get_vector();
				std::cout << content.size() << " bytes received" <<  std::endl;;
				size_t headerSize = ProtobufPacket<robogenMessage::EvaluationRequest>::HEADER_SIZE;
				std::vector<unsigned char> headerBuffer;
				for (unsigned int i = 0; i < headerSize; ++i) {
					headerBuffer.push_back(content.at(i)->get_int());
				}

				ProtobufPacket<robogenMessage::EvaluationRequest> packet;
				unsigned int packSize = packet.decodeHeader(headerBuffer);

				std::vector<unsigned char> payloadBuffer;
				for (unsigned int i = headerSize; i < content.size(); ++i) {
					payloadBuffer.push_back(content.at(i)->get_int());
				}

				std::cout << payloadBuffer.size() << std::endl;
				packet.decodePayload(payloadBuffer);

				std::cout << "packet decoded" << std::endl;
				


					boost::shared_ptr<RobogenConfig> configuration =
							ConfigurationReader::parseRobogenMessage(
									packet.getMessage()->configuration());
					if (configuration == NULL) {
						std::cerr
								<< "Problems parsing the configuration file. Quit."
								<< std::endl;
						exitRobogen(EXIT_FAILURE);
					}

					// ---------------------------------------
					// Setup environment
					// ---------------------------------------

					boost::shared_ptr<Scenario> scenario =
							ScenarioFactory::createScenario(configuration);
					if (scenario == NULL) {
						exitRobogen(EXIT_FAILURE);
					}

					std::cout
							<< "-----------------------------------------------"
							<< std::endl;

					// ---------------------------------------
					// Run simulations
					// ---------------------------------------

					unsigned int simulationResult = runSimulations(scenario,
							configuration, packet.getMessage()->robot(),
							nullptr, rng);


					if (simulationResult == SIMULATION_FAILURE) {
						exitRobogen(EXIT_FAILURE);
					}

					// ---------------------------------------
					// Compute fitness
					// ---------------------------------------
					double fitness;
					if (simulationResult == CONSTRAINT_VIOLATED) {
						fitness = MIN_FITNESS;
					} else {
						fitness = scenario->getFitness();
					}

					message::ptr output = object_message::create();
					output->get_map()["id"] = string_message::create(id);
					output->get_map()["content"] = object_message::create();
					output->get_map()["content"]->get_map()["fitness"] = double_message::create(fitness);
					output->get_map()["content"]->get_map()["ptr"] = data->get_map()["content"]->get_map()["ptr"];
					current_socket->emit("responseTask", output);
					std::cout << "Fitness for the current solution: " << fitness
							<< std::endl << std::endl;


				_lock.unlock();
				}));
}

int main(int argc ,const char* args[])
{

	rng.seed(3000);
	sio::client h;
	connection_listener l(h);

	h.set_open_listener(std::bind(&connection_listener::on_connected, &l));
	h.set_close_listener(std::bind(&connection_listener::on_close, &l,std::placeholders::_1));
	h.set_fail_listener(std::bind(&connection_listener::on_fail, &l));
	if (argc < 2) {
		std::cerr << "you must provide a server url" << std::endl;
		exit(1);
	}
	h.connect(args[1]);
	current_socket = h.socket();
	bind_events(current_socket);
	_lock.lock();
	if(!connect_finish)
	{
		_cond.wait(_lock);
	}
	_lock.unlock();
	message::ptr capabilities = object_message::create();
	// declare capabilities (1 thread)
	capabilities->get_map()["totalNodes"] = int_message::create(1);
	current_socket->emit("computationDeclaration", capabilities);
	_lock.lock();
	_cond.wait(_lock);
	_lock.unlock();
	h.sync_close();
	h.clear_con_listeners();
	return 0;
}
