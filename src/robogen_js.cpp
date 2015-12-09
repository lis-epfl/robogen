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
#include <scenario/JSScenario.h>

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
	std::cout << "A robogenJS worker has started, he is waiti"
			"ng for any task"
			<< std::endl;
	return 0;
}

#ifdef EMSCRIPTEN
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
	if (!scenario) {
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
	if (simulationResult == CONSTRAINT_VIOLATED) {
		fitness = MIN_FITNESS;
	} else {
		fitness = scenario->getFitness();
	}
	std::cout << "Fitness for the current solution: " << fitness
	<< std::endl << std::endl;
	return fitness;
}

struct ScenarioWrapper : public emscripten::wrapper<JSScenario> {
    EMSCRIPTEN_WRAPPER(ScenarioWrapper);
    double getFitness() {
        return call<double>("getFitness");
    }

    bool endSimulationJS() {
    	return call<bool>("endSimulation");
    }

    bool setupSimulation() {
    	return call<bool>("setupSimulation");
    }
};


std::vector<float> testReturnVector() {
	std::vector<float> a;
	a.push_back(2.3);
	a.push_back(4.5);
	return a;
}


EMSCRIPTEN_BINDINGS(my_module) {
	emscripten::function("simulationViewer", &simulationViewer);
	emscripten::function("runEvolution", &runEvolution);
	emscripten::function("evaluate", &evaluate);
	emscripten::function("evaluationResultAvailable", &evaluationResultAvailable);
	emscripten::function("evaluationIsDone", &evaluationIsDone);

	emscripten::register_vector<float>("FloatVector");
	emscripten::register_vector<boost::shared_ptr<Model> >("ModelVector");
	emscripten::function("testReturnVector", &testReturnVector);

	emscripten::class_<JSScenario>("JSScenario")
        .function("getFitness", &JSScenario::getFitness, emscripten::pure_virtual())
        .function("setupSimulation", &JSScenario::setupSimulation, emscripten::pure_virtual())
        .function("endSimulation", &JSScenario::endSimulationJS, emscripten::pure_virtual())
        .function("setId", &JSScenario::setId)
        .function("getId", &JSScenario::getId)
        //.function("getRobot", )
        .function("getRobotPosition", &JSScenario::getRobotPosition)
        .function("printRobotPosition", &JSScenario::printRobotPosition)
        .allow_subclass<ScenarioWrapper>("ScenarioWrapper");

	emscripten::class_<Robot>("Robot")
		.function("getBodyParts", &Robot::getBodyParts);

}
#endif

