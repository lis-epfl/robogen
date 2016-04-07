/*
 * @(#) QScriptScenario.cpp   1.0   Dec 22, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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

#ifdef QT5_ENABLED

#include "QScriptScenario.h"
#include <osg/Vec3>
#include "config/RobogenConfig.h"
#include "config/StartPositionConfig.h"
#include "scenario/Environment.h"
#include "Robot.h"

#ifdef WIN32
    #ifndef NAN
        static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
        #define NAN (*(const float *) __nan)
    #endif
#endif

namespace robogen {

QScriptScenario::QScriptScenario(boost::shared_ptr<RobogenConfig> config) :
	Scenario(config), engine_(new QScriptEngine), curTrial_(0) {

	engine_->globalObject().setProperty("qScriptScenario",
			engine_->newQObject(this));

	// add custom stuff to user provided script (no newlines so that
	// errors will report proper line!)
	std::stringstream ss;
	ss << "function extend(target, source){"
	   <<	"for(prop in source){"
	   <<      "target[prop] = source[prop];"
	   <<   "}"
	   <<  "} ";

	ss << "var userScenario = ";

	std::string userCode = config->getScenario();

	const std::string from = "console.log";
	const std::string to = "print";

	std::string::size_type n = 0;
	while ( ( n = userCode.find( from, n ) ) != std::string::npos )
	{
		userCode.replace( n, from.size(), to );
	    n += to.size();
	}

	ss << userCode;
	ss << "\n";
	// here we extend our scenario with the user's stuff
	ss << "extend(qScriptScenario, userScenario);\n";

	/*// set the prototype of the user's scenario to be
	// our instance of QScriptScenario
	ss << "UserScenario.prototype = qScriptScenario;";
	*/
	// evaluate the script
	QScriptProgram program(QString::fromStdString(ss.str()));
	QScriptValue result = engine_->evaluate(program);

	if(result.isError()) {
		std::cerr << std::endl << "Problem with the provided scenario script!"
				<< std::endl << std::endl
				<< engine_->uncaughtException().toString().toStdString()
				<< " at line " << engine_->uncaughtExceptionLineNumber()
				<< std::endl << std::endl;
		QStringList backtrace = engine_->uncaughtExceptionBacktrace();
		for(int i = 0; i < backtrace.size(); ++i) {
			std::cerr << backtrace[i].toStdString() << std::endl;
		}
		std::cerr << std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	userScenario_ = engine_->globalObject().property("qScriptScenario");

	// since we are faking our interface, let's check which methods have
	// been implemented

	// first check for getFitness, the only required method
	if(!isValidFunction("getFitness")) {
		std::cerr << "The provided scenario script must implement getFitness!"
				<< std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	// now check all (include getFitness, so user does not get confused
	// 				  by its omission)
	std::string methods[] = {"setupSimulation", "afterSimulationStep",
								"endSimulation", "getFitness" };
	size_t numMethods = 4;

	for(size_t i = 0; i < numMethods; ++i) {
		implementedMethods_[methods[i]] = isValidFunction(methods[i]);
		if (implementedMethods_[methods[i]]) {
			std::cout << "Using the provided " << methods[i] << " method"
					<< std::endl;
		}
	}


}

QScriptScenario::~QScriptScenario() {
}

bool QScriptScenario::setupSimulation() {

	// set up exposed stuff before user's setup
	qRobot_ = engine_->newQObject(new qscript::QRobot(Scenario::getRobot()),
			QScriptEngine::ScriptOwnership);
	qEnvironment_ = engine_->newQObject(
			new qscript::QEnvironment(Scenario::getEnvironment()),
			QScriptEngine::ScriptOwnership);

	if(implementedMethods_["setupSimulation"]) {
		QScriptValue function = userScenario_.property("setupSimulation");
		QScriptValue resultValue = function.call(userScenario_);
		if(engine_->hasUncaughtException()) {
			std::cerr << resultValue.toString().toStdString() << std::endl;
			return false;
		} else {
			return resultValue.toBool();
		}
	}


	return true;
}

bool QScriptScenario::afterSimulationStep() {
	if(implementedMethods_["afterSimulationStep"]) {
		QScriptValue function = userScenario_.property("afterSimulationStep");
		QScriptValue resultValue = function.call(userScenario_);
		if(engine_->hasUncaughtException()) {
			std::cerr << resultValue.toString().toStdString() << std::endl;
			return false;
		} else {
			return resultValue.toBool();
		}
	}
	return true;
}

bool QScriptScenario::endSimulation() {
	bool result = true;
	if(implementedMethods_["endSimulation"]) {
		QScriptValue function = userScenario_.property("endSimulation");
		QScriptValue resultValue = function.call(userScenario_);
		if(engine_->hasUncaughtException()) {
			std::cerr << resultValue.toString().toStdString() << std::endl;
			result = false;
		} else {
			result = resultValue.toBool();
		}
	}

	curTrial_++;
	this->setStartingPosition(curTrial_);

	return result;
}

bool QScriptScenario::remainingTrials() {
	boost::shared_ptr<StartPositionConfig> startPos = this->getRobogenConfig()->getStartingPos();
	return curTrial_ < startPos->getStartPosition().size();
}

int QScriptScenario::getCurTrial() const {
	return curTrial_;
}



double QScriptScenario::getFitness() {
	QScriptValue function = userScenario_.property("getFitness");
	QScriptValue resultValue = function.call(userScenario_);
	if(engine_->hasUncaughtException()) {
		std::cerr << resultValue.toString().toStdString() << std::endl;
		return NAN;
	} else {
		return resultValue.toNumber();
	}
}

bool QScriptScenario::isValidFunction(std::string name) {
	QScriptValue function = userScenario_.property(name.c_str());
	return function.isValid();
}

float QScriptScenario::vectorDistance(QScriptValue vector1,
		QScriptValue vector2) {
	if (vector1.property("x").isNumber() && vector2.property("x").isNumber()) {

		if (vector1.property("y").isNumber() &&
				vector2.property("y").isNumber()) {

			if (vector1.property("z").isNumber() &&
					vector2.property("z").isNumber()) {
				return (osg::Vec3(vector1.property("x").toNumber(),
							vector1.property("y").toNumber(),
							vector1.property("z").toNumber()) -
						osg::Vec3(vector2.property("x").toNumber(),
							vector2.property("y").toNumber(),
							vector2.property("z").toNumber())).length();
			} else if(vector1.property("z").isNumber() ||
					vector2.property("z").isNumber()) {
				std::cerr << "Problem computing vector distance!  Both vectors"
						<< " must have same dimension!!" << std::endl;
				exitRobogen(EXIT_FAILURE);
			} else {
				return (osg::Vec2(vector1.property("x").toNumber(),
							vector1.property("y").toNumber()) -
						osg::Vec2(vector2.property("x").toNumber(),
							vector2.property("y").toNumber())).length();
			}
		} else if(vector1.property("y").isNumber() ||
				vector2.property("y").isNumber()) {
			std::cerr << "Problem computing vector distance!  Both vectors"
					<< " must have same dimension!!" << std::endl;
			exitRobogen(EXIT_FAILURE);
		} else {
			return fabs(vector1.property("x").toNumber() -
					vector2.property("x").toNumber());
		}
	}

	std::cerr << "Problem computing vector distance!  Must provide two "
			"equal dimensional vectors!!" << std::endl;
	exitRobogen(EXIT_FAILURE);
	return NAN;
}


}

#endif
