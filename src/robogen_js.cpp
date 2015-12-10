/*
 * robogen_js.cpp
 *
 *  Created on: Aug 11, 2015
 *      Author: guillaume3
 */
#include <iostream>
#include <emscripten/bind.h>
#include <emscripten.h>
#include "utils/JSUtils.h"

#include <viewer/JSViewer.h>
#include <scenario/JSScenario.h>

#include "model/objects/BoxObstacle.h"

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

    bool afterSimulationStep() {
    	return call<bool>("afterSimulationStep");
    }


};

#define TEST_EM
#ifdef TEST_EM
#include "emscripten_test.cpp"
#endif


// helper functions
emscripten::val getModelRootPosition(boost::shared_ptr<Model> model) {
	return js::valFromVec3(model->getRootPosition());
}
emscripten::val getModelRootAttitude(boost::shared_ptr<Model> model) {
	return js::valFromQuat(model->getRootAttitude());
}

emscripten::val getObservablePosition(boost::shared_ptr<PositionObservable>
		observable) {
	return js::valFromVec3(observable->getPosition());
}

emscripten::val getObservableAttitude(boost::shared_ptr<PositionObservable>
		observable) {
	return js::valFromQuat(observable->getAttitude());
}

emscripten::val getBoxSize(boost::shared_ptr<BoxObstacle> boxObstacle) {
	return js::valFromVec3(boxObstacle->getSize());
}



emscripten::val getMotorId(boost::shared_ptr<Motor> motor) {
	emscripten::val result(emscripten::val::object());
	ioPair id = motor->getId();
	result.set("partId", id.first);
	result.set("ioId", id.second);
	return result;
}


EMSCRIPTEN_BINDINGS(my_module) {
	emscripten::function("simulationViewer", &simulationViewer);
	emscripten::function("runEvolution", &runEvolution);
	emscripten::function("evaluate", &evaluate);
	emscripten::function("evaluationResultAvailable", &evaluationResultAvailable);
	emscripten::function("evaluationIsDone", &evaluationIsDone);

	emscripten::register_vector<float>("FloatVector");
	emscripten::register_vector<boost::shared_ptr<Model> >("ModelVector");
	emscripten::register_vector<boost::shared_ptr<Sensor> >("SensorVector");
	emscripten::register_vector<boost::shared_ptr<Motor> >("MotorVector");
	emscripten::register_vector<boost::shared_ptr<Robot> >("Robot");
	emscripten::register_vector<boost::shared_ptr<Robot> >("LightSources");

	emscripten::class_<Model>("Model")
		.function("getRootPosition", &getModelRootPosition)
		.function("getRootAttitude", &getModelRootAttitude)
		.smart_ptr<boost::shared_ptr<Model> >("shared_ptr<Model>");

	emscripten::class_<Sensor>("Sensor")
		.smart_ptr<boost::shared_ptr<Sensor> >("shared_ptr<Sensor>")
		.function("getLabel", &Sensor::getLabel)
		.function("read", &Sensor::read)
		;

	emscripten::class_<Motor>("Motor")
		.smart_ptr<boost::shared_ptr<Motor> >("shared_ptr<Motor>")
		.function("getId", &getMotorId);

	emscripten::class_<ServoMotor, emscripten::base<Motor>>("ServoMotor")
		.smart_ptr<boost::shared_ptr<ServoMotor> >("shared_ptr<ServoMotor>")
		.function("isVelocityDriven", &ServoMotor::isVelocityDriven)
		.function("getVelocity", &ServoMotor::getVelocity)
		.function("getPosition", &ServoMotor::getPosition)
		.function("getTorque", &ServoMotor::getTorque)
		;

	emscripten::class_<Robot>("Robot")
		.smart_ptr<boost::shared_ptr<Robot> >("shared_ptr<Robot>")
		.function("getBodyParts", &Robot::getBodyParts)
		.function("getCoreComponent", &Robot::getCoreComponent)
		.function("getSensors", &Robot::getSensors)
		;


	emscripten::class_<Scenario>("Scenario")
		.function("getRobot", &Scenario::getRobot)
		.function("getEnvironment", &Scenario::getEnvironment)
		;

	emscripten::class_<JSScenario, emscripten::base<Scenario>>("JSScenario")
        .function("getFitness", &JSScenario::getFitness, emscripten::pure_virtual())
        .function("afterSimulationStep",
        		emscripten::optional_override([](JSScenario& self) {
							return self.JSScenario::afterSimulationStep();
						}))
		.function("setupSimulation",
				emscripten::optional_override([](JSScenario& self) {
							return self.JSScenario::setupSimulation();
						}))
		.function("endSimulation",
				emscripten::optional_override([](JSScenario& self) {
							return self.JSScenario::endSimulationJS();
						}))
        .function("setId", &JSScenario::setId)
        .function("getId", &JSScenario::getId)
        .function("printRobotPosition", &JSScenario::printRobotPosition)
        .allow_subclass<ScenarioWrapper>("ScenarioWrapper")
		;

	emscripten::class_<PositionObservable>("PositionObservable")
		.function("getPosition", &getObservablePosition)
		.function("getAttitude", &getObservableAttitude)
		;

	emscripten::class_<LightSource,  emscripten::base<PositionObservable>>("LightSource")
		.smart_ptr<boost::shared_ptr<LightSource> >("shared_ptr<LightSource>")
		.function("getIntensity", &LightSource::getIntensity)
		;

	emscripten::class_<Obstacle,  emscripten::base<PositionObservable>>("Obstacle")
		.smart_ptr<boost::shared_ptr<Obstacle> >("shared_ptr<Obstacle>")
		;

	emscripten::class_<BoxObstacle,  emscripten::base<Obstacle>>("BoxObstacle")
		.smart_ptr<boost::shared_ptr<BoxObstacle> >("shared_ptr<BoxObstacle>")
		.function("getSize", &getBoxSize)
		;

	emscripten::class_<Environment>("Environment")
		.smart_ptr<boost::shared_ptr<Environment> >("shared_ptr<Environment>")
		//.function("getTimeElapsed")
		.function("getLightSources", &Environment::getLightSources)
		.function("getAmbientLight", &Environment::getAmbientLight)
		.function("getObstacles", &Environment::getObstacles)
		;

#ifdef TEST_EM

	emscripten::class_<Base>("Base")
		.function("doSomething", &Base::doSomething);
		;


	emscripten::class_<Derived, emscripten::base<Base>>("Derived")
	    .constructor()
	    ;

	emscripten::function("testReturnVec3", &testReturnVec3);

	emscripten::function("testReturnVector", &testReturnVector);
	emscripten::function("testReturnSharedPtr", &testReturnSharedPtr);
	emscripten::function("testReturnRawPtr", &testReturnRawPtr, emscripten::allow_raw_pointers());
	emscripten::function("runEmBindTest", &runEmBindTest);
	emscripten::function("printTestStruct", &printTestStruct, emscripten::allow_raw_pointers());

	emscripten::class_<TestStruct>("TestStruct")
			//.constructor<float>()
			.property("a", &TestStruct::a)
			.smart_ptr<boost::shared_ptr<TestStruct> >("shared_ptr<TestStruct>");
#endif

}
#endif

