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

int main(int argc, char** argv) {
	if (argc < 2) {
		qDebug() << "Usage : " << argv[0] << " scriptPath";
		return 0;
	}
	QString script = readFile(argv[1]);

	// app + engine init
	QCoreApplication a(argc, argv);
	QScriptEngine engine;

	// creating an instance of the API exposing class
	boost::shared_ptr<QObject> stuff(new Exposable());
	QScriptValue objectValue = engine.newQObject(stuff.get());

	// assigning it to the global variable "myAPI" in the javascript code
	engine.globalObject().setProperty("myAPI", objectValue);

	// reading / precompiling the script
	QScriptProgram program(script);
	engine.evaluate(program);

	// calling methods from the script
	qDebug() << "fitness = " << engine.evaluate("getFitness()").toNumber();
	return 0;
}
