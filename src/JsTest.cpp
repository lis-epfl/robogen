/*
 * @(#) JsTest.cpp   1.0   Dec 3, 2015
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

#include "Exposable.h"
#include <QCoreApplication>
#include <QScriptEngine>
#include <QDebug>
#include <memory>
#include <QFile>
#include <QString>
#include <QTextStream>
#include <boost/shared_ptr.hpp>

#include <iostream>


Q_DECLARE_METATYPE(boost::shared_ptr<Exposable>)

/*QScriptValue exposableToScriptValue(QScriptEngine *engine, boost::shared_ptr<Exposable> const &in) {
	return engine->newQObject(in.get());
}

void exposableFromScriptValue(const QScriptValue &object, boost::shared_ptr<Exposable> &out) {
	out = qobject_cast<Exposable*>(object.toQObject())->getShared();
}*/



// just a utility function to read a file...
QString readFile(const QString& path) {
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
		qDebug() << "unable to open file";
		return "";
	}
	QTextStream in(&f);
	return in.readAll();
}

QScriptValue Exposable_ctor(QScriptContext *ctx, QScriptEngine *eng)
{
    return eng->newQObject(new Exposable());
}

int main(int argc, char** argv) {
	if (argc < 2) {
		qDebug() << "Usage : " << argv[0] << " scriptPath";
		return 0;
	}
	QString script = readFile(argv[1]);

	// app + engine init
	QCoreApplication a(argc, argv);
	QScriptEngine engine;

	//qScriptRegisterMetaType(&engine, exposableToScriptValue, exposableFromScriptValue);

	// creating an instance of the API exposing class
	//boost::shared_ptr<QObject> stuff(new Exposable());

	//QScriptValue module = engine.newObject();

	// assigning it to the global variable "myAPI" in the javascript code
	//engine.globalObject().setProperty("Module", module);
	//module.setProperty("myAPI", engine.newQObject(stuff.get()));
	//boost::shared_ptr<QObject> interface(new Interface());
	//module.setProperty("Interface", engine.newQObject(interface.get()));


	//QScriptValue exposableClass = engine.scriptValueFromQMetaObject<Exposable>();

	Exposable* exposable = new Exposable();

	engine.globalObject().setProperty("Exposable", engine.newQObject(exposable));//engine.newFunction(Exposable_ctor));

	//engine.globalObject().setProperty("TestType", engine.newFunction(myQObjectConstructor));
	// reading / precompiling the script
	QScriptProgram program(script);
	engine.evaluate(program);

	//std::cout << "a" << std::endl;

	QScriptValue userObject = engine.globalObject().property("Exposable");//.construct();



//	std::cout << userObject.property("invoke").isValid() << std::endl;

	exposable->setA(99);
	exposable->setUserObject(userObject);
	exposable->invoke();

	userObject.setProperty("foo", 27);

	exposable->setA(103);
	exposable->invoke();



	//boost::shared_ptr<Exposable> userImpl(new Exposable());

	//userImpl->setA(999);

	//std::cout << "c" << std::endl;

	//userImpl->setUserObject(userObject);
	//userImpl->invoke();






	return 0;
}
