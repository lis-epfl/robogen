/*
 * @(#) QScriptScenario.h   1.0   Dec 22, 2015
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
#ifndef ROBOGEN_QSCRIPTSCENARIO_H_
#define ROBOGEN_QSCRIPTSCENARIO_H_

#ifdef QT5_ENABLED

#include <map>
#include <QScriptEngine>
#include "Scenario.h"

#include "utils/QScriptUtils.h"
#include "scripting/QBindings.h"

namespace robogen {




class QScriptScenario :  public QObject, public Scenario {
	Q_OBJECT

public:

	QScriptScenario(boost::shared_ptr<RobogenConfig> config);


	/**
	 * Destructor
	 */
	virtual ~QScriptScenario();

	/**
	 * Methods inherited from {@link #Scenario}
	 */

	// for now do nothing on any of these except getFitness, which
	// will be pure virtual here
	virtual bool setupSimulation();
	virtual bool afterSimulationStep();
	virtual bool endSimulation();
	virtual double getFitness();
	virtual bool remainingTrials();


public slots:
	QScriptValue getRobot() { return qRobot_; }
	QScriptValue getEnvironment() { return qEnvironment_; }
	virtual int getCurTrial() const;
	float vectorDistance(QScriptValue vector1, QScriptValue vector2);


private:
	bool isValidFunction(std::string name);


	// keep shared ptr to QScriptEngine so it stays alive for life of scenario
	boost::shared_ptr<QScriptEngine> engine_;
	std::string id_;
	unsigned int curTrial_;
	QScriptValue userScenario_;
	std::map<std::string, bool> implementedMethods_;

	QScriptValue qRobot_, qEnvironment_;

};

}

#endif

#endif /* ROBOGEN_QSCRIPTSCENARIO_H_ */
