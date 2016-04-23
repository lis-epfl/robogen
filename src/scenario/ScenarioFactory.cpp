/*
 * @(#) ScenarioFactory.cpp   1.0   Mar 13, 2013
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
#include <iostream>
#include <sstream>
#include "config/RobogenConfig.h"
#include "scenario/ChasingScenario.h"
#include "scenario/ScenarioFactory.h"
#include "scenario/RacingScenario.h"

#ifdef EMSCRIPTEN
#include <emscripten/bind.h>
#include <emscripten.h>
#include "scenario/JSScenario.h"
#include "utils/JSUtils.h"

#include <algorithm>






#elif defined(QT5_ENABLED)
#include <QScriptEngine>
#include "QScriptScenario.h"
#endif





namespace robogen {

ScenarioFactory::ScenarioFactory() {

}

ScenarioFactory::~ScenarioFactory() {

}

boost::shared_ptr<Scenario> ScenarioFactory::createScenario(boost::shared_ptr<RobogenConfig> config) {

	if (config->getScenario() == "racing") {
		return boost::shared_ptr<Scenario>(new RacingScenario(config));
	} else if (config->getScenario() == "chasing") {
		return boost::shared_ptr<Scenario>(new ChasingScenario(config));
	} else {
		// we are getting scenario in js
#ifdef EMSCRIPTEN

		return JSScenario::createScenario(config);

#elif defined(QT5_ENABLED)
    	std::cout << "Qt5 enabled, using scripted scenario!" << std::endl;
    	return boost::shared_ptr<Scenario>(new QScriptScenario(config));


#else
		std::cout << "JS scenarios are not available in C++ version w/out Qt5!" << std::endl;
		std::cout << config->getScenario() << std::endl;

#endif
	}

	return boost::shared_ptr<Scenario>();

}

}
