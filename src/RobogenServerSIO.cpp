/*
 * @(#) RobogenServerSIO.cpp   1.0   April 24, 2016
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc (guillaume.leclerc@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach, Guillaume Leclerc
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


#include <chrono>
#include <ctime>

#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>


#include "sio_client.h"
#include "sio_message.h"
#include "sio_socket.h"


#ifdef QT5_ENABLED
#include <QCoreApplication>
#define EMIT_TMP emit
#undef emit
#endif



using namespace robogen;


// ODE World
dWorldID odeWorld;

// Container for collisions
dJointGroupID odeContactGroup;

bool interrupted;


std::mutex _lock;

std::condition_variable_any _cond;
bool connectFinish = false;

class ConnectionListener {

public:

	ConnectionListener(sio::client& handler) : handler_(handler) {
	}


	void on_connected() {
		_lock.lock();
		_cond.notify_all();
		connectFinish = true;
		_lock.unlock();
	}
	void on_close(sio::client::close_reason const& reason) {
		std::cout<<"sio closed "<<std::endl;
		exit(0);
	}

	void on_fail() {
		std::cout<<"sio failed "<<std::endl;
		exit(0);
	}

private:
	sio::client &handler_;
};

int participants = -1;

boost::random::mt19937 rng;
sio::socket::ptr currentSocket;


void bind_events(sio::socket::ptr &socket) {
	currentSocket->on("requestTask", sio::socket::event_listener_aux([&](
			std::string const& name, sio::message::ptr const& data,
			bool isAck,sio::message::list &ack_resp) {
				_lock.lock();
				const std::string id = data->get_map()["id"]->get_string();
				std::cout << "I have a new task (id:" << id << ")" << std::endl;
				std::vector<sio::message::ptr> content =
						data->get_map()["content"]->get_map(
								)["packet"]->get_vector();
				std::cout << content.size() << " bytes received" <<  std::endl;;
				size_t headerSize = ProtobufPacket<
						robogenMessage::EvaluationRequest>::HEADER_SIZE;
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

					sio::message::ptr output = sio::object_message::create();
					output->get_map()["id"] = sio::string_message::create(id);
					output->get_map()["content"] =
							sio::object_message::create();
					output->get_map()["content"]->get_map()["fitness"] =
							sio::double_message::create(fitness);
					output->get_map()["content"]->get_map()["ptr"] =
							data->get_map()["content"]->get_map()["ptr"];
					currentSocket->emit("responseTask", output);
					std::cout << "Fitness for the current solution: " << fitness
							<< std::endl << std::endl;


				_lock.unlock();
				}));
}

int main(int argc, char* argv[]) {

#ifdef QT5_ENABLED
	QCoreApplication a(argc, argv);
#endif

	//rng.seed(3000);
	rng.seed(std::time(0));
	sio::client handler;
	ConnectionListener listener(handler);

	handler.set_open_listener(std::bind(&ConnectionListener::on_connected,
										&listener));
	handler.set_close_listener(std::bind(&ConnectionListener::on_close,
										&listener, std::placeholders::_1));
	handler.set_fail_listener(std::bind(&ConnectionListener::on_fail,
										&listener));
	if (argc < 3) {
		std::cerr << "you must provide a server url and group(s)" << std::endl;
		exit(1);
	}
	handler.connect(argv[1]);
	currentSocket = handler.socket();
	bind_events(currentSocket);
	_lock.lock();
	if(!connectFinish)
	{
		_cond.wait(_lock);
	}
	_lock.unlock();
	sio::message::ptr capabilities = sio::object_message::create();
	// declare capabilities (1 thread)
	capabilities->get_map()["totalNodes"] = sio::int_message::create(1);
	currentSocket->emit("computationDeclaration", capabilities);
	for (int i=2; i<argc; ++i) {
		sio::message::ptr group = sio::object_message::create();
		group->get_map()["id"] = sio::string_message::create(argv[i]);
		currentSocket->emit("joinGroup", group);
		std::cout << "Join group " << argv[i] << std::endl;
	}
	_lock.lock();
	_cond.wait(_lock);
	_lock.unlock();
	handler.sync_close();
	handler.clear_con_listeners();
	return 0;
}

#ifdef QT5_ENABLED
#define emit EMIT_TMP
#endif

