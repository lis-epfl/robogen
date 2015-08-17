/*
 * robogen_js.cpp
 *
 *  Created on: Aug 11, 2015
 *      Author: guillaume3
 */
#include <iostream>
#include <emscripten/bind.h>
#include <emscripten.h>
#include <viewer/JSViewer.h>

/***** WARNING ******
 * WE ARE INCLUDING .cpp files !!
 * This file is "merging" cpp files and expose functions to the javascript world
 ********************/

void sendJSEvent(std::string name, std::string jsonData) {

	std::string command = "self.cppEvent(\"";
	command += name;
	command += "\", ";
	command += jsonData;
	command += ")";
#ifdef EMSCRIPTEN
	emscripten_run_script(command.c_str());
#else
	std::cout << command << std::endl;
#endif
}

int main(int argc, char** argv) {
	std::cout << "A robogenJS worker has started, he is waiting for any task"
			<< std::endl;
	return 0;
}
#include <Evolver.cpp>
#include <viewer/FileViewer.cpp>

double EMSCRIPTEN_KEEPALIVE evaluate(int ptr, int length) {
	unsigned char* data = (unsigned char*) ptr;
	boost::random::mt19937 rng;
	rng.seed(ptr);

	ProtobufPacket<robogenMessage::EvaluationRequest> packet;
	std::vector<unsigned char> payloadBuffer;
	for (unsigned int i = ProtobufPacket<robogenMessage::EvaluationRequest>::HEADER_SIZE ; i < length; ++i) {
		payloadBuffer.push_back(data[i]);
	}
	packet.decodePayload(payloadBuffer);
	// ---------------------------------------
	//  Decode configuration file
	// ---------------------------------------

	boost::shared_ptr<RobogenConfig> configuration =
	ConfigurationReader::parseRobogenMessage(
			packet.getMessage()->configuration());
	if (configuration == NULL) {
		std::cerr
		<< "Problems parsing the configuration file. Quit."
		<< std::endl;
		return -1;
	}

	// ---------------------------------------
	// Setup environment
	// ---------------------------------------

	boost::shared_ptr<Scenario> scenario =
	ScenarioFactory::createScenario(configuration);
	if (scenario == NULL) {
		return -1;
	}

	std::cout
	<< "-----------------------------------------------"
	<< std::endl;

	// ---------------------------------------
	// Run simulations
	// ---------------------------------------

	JSViewer* viewer = new JSViewer();

	unsigned int simulationResult = runSimulations(scenario,
			configuration, packet.getMessage()->robot(),
			viewer, rng);

	delete viewer;

	if (simulationResult == SIMULATION_FAILURE) {
		return -1;
	}

	// ---------------------------------------
	// Compute fitness
	// ---------------------------------------
	double fitness;
	if (simulationResult == ACCELERATION_CAP_EXCEEDED) {
		fitness = MIN_FITNESS;
	} else {
		fitness = scenario->getFitness();
	}
	std::cout << "Fitness for the current solution: " << fitness
	<< std::endl << std::endl;
	return fitness;
}

EMSCRIPTEN_BINDINGS(my_module) {
	emscripten::function("simulationViewer", &simulationViewer);
	emscripten::function("runEvolution", &runEvolution);
	emscripten::function("evaluate", &evaluate);
	emscripten::function("evaluationResultAvailable", &evaluationResultAvailable);
}

