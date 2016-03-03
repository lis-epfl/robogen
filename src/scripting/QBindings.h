/*
 * @(#) QBindings.h   1.0   Dec 22, 2015
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


#ifndef ROBOGEN_QBINDINGS_H_
#define ROBOGEN_QBINDINGS_H_

#ifdef QT5_ENABLED

#include <QScriptEngine>
#include <QScriptable>
#include <boost/weak_ptr.hpp>
#include "Robot.h"
#include "model/motors/Motor.h"
#include "model/PositionObservable.h"
#include "scenario/Environment.h"
#include "model/objects/BoxObstacle.h"

// note we keep weak pointers here, so that they don't keep objects alive
// after the scenario has been pruned


namespace robogen {
namespace qscript {


class QMotor : public QObject, public QScriptable {
	Q_OBJECT
public:
	QMotor(boost::weak_ptr<Motor> basePtr);

public slots:
	QScriptValue getId();
	QScriptValue getVelocity();
	QScriptValue getPosition();
	QScriptValue getTorque();

protected:
	QScriptValue id_;
	boost::weak_ptr<Motor> basePtr_;
};

class QSensor : public QObject, public QScriptable {
	Q_OBJECT
public:
	QSensor(boost::weak_ptr<Sensor> basePtr);
public slots:
	QScriptValue getLabel();
	QScriptValue read();
private:
	boost::weak_ptr<Sensor> basePtr_;
};


class QModel : public QObject, public QScriptable {
	Q_OBJECT
public:
	QModel(boost::weak_ptr<Model> basePtr);

public slots:
	QScriptValue getRootPosition();
	QScriptValue getRootAttitude();
	QScriptValue getType();

private:
	boost::weak_ptr<Model> basePtr_;
};

class QRobot :  public QObject, public QScriptable {
	Q_OBJECT
public:
	QRobot(boost::weak_ptr<Robot> basePtr);

public slots:
	//QScriptValue getPosition();
	QScriptValue getBodyParts();
	QScriptValue getCoreComponent();
	QScriptValue getMotors();
	QScriptValue getSensors();


private :
	boost::weak_ptr<Robot> basePtr_;

	QScriptValue bodyParts_;
	QScriptValue coreComponent_;
	QScriptValue motors_;
	QScriptValue sensors_;
};

class QPositionObservable :  public QObject, public QScriptable {
	Q_OBJECT
public:
	QPositionObservable(boost::weak_ptr<PositionObservable> basePtr);
public slots:
	virtual QScriptValue getPosition();
	virtual QScriptValue getAttitude();
protected:
	boost::weak_ptr<PositionObservable> basePtr_;
};

class QLightSource : public QPositionObservable {
	Q_OBJECT
public :
	QLightSource(boost::weak_ptr<LightSource> basePtr) :
		QPositionObservable(basePtr) {}
public slots:
	QScriptValue getIntensity();
	void setPosition(float x, float y, float z);
	void setIntensity(float intensity);
};

class QObstacle : public QPositionObservable {
	Q_OBJECT
public :
	QObstacle(boost::weak_ptr<Obstacle> basePtr) :
		QPositionObservable(basePtr) {}
};

class QBoxObstacle : public QObstacle {
	Q_OBJECT
public :
	QBoxObstacle(boost::weak_ptr<BoxObstacle> basePtr) :
		QObstacle(basePtr) {}
public slots:
	QScriptValue getSize();
};

class QEnvironment : public QObject, public QScriptable {
	Q_OBJECT
public:
	QEnvironment(boost::weak_ptr<Environment> basePtr);
public slots:
	QScriptValue getLightSources();
	QScriptValue getAmbientLight();
	QScriptValue getObstacles();
private:
	boost::weak_ptr<Environment> basePtr_;
	QScriptValue lightSources_;
	QScriptValue obstacles_;
};




}
}


#endif

#endif
