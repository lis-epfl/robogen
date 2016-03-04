/*
 * @(#) Exposable.h   1.0   Dec 3, 2015
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
#ifndef EXPOSABLE_H_
#define EXPOSABLE_H_

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <memory>
#include <QCoreApplication>
#include <QScriptEngine>
#include <QScriptable>

class Exposable : public QObject, public QScriptable,
					public boost::enable_shared_from_this<Exposable> {
	Q_OBJECT
	int a_, b_;
	boost::shared_ptr<Exposable> child_;
	QScriptValue userObject_;

 public:
	Exposable() : a_(5), b_(7), child_(new Exposable(2, 3)) {}
	Exposable(int a, int b) : a_(a), b_(b) {}
	/*Exposable(const Exposable& exposable) :
		QObject(), boost::enable_shared_from_this<Exposable>(),
		a(exposable.a), b(exposable.b), child(exposable.child) {
	}*/


	~Exposable() {}
	boost::shared_ptr<Exposable> getShared() { return shared_from_this(); }
	void setUserObject(QScriptValue userObject) {
		userObject_ = userObject;
	}

	void invoke() { //const std::string& str) {
		QScriptValue function = userObject_.property("invoke");
		if(!function.isValid()) {
			std::cout << "NOT DEFINED!!" << std::endl;
		}
		function.call(userObject_);
	}
	void setA(int a) { a_ = a; }


 public slots:
	int getA() { return a_; }
	int getB() { return b_; }

	QScriptValue getBar() {
		QScriptValue result = engine()->newObject();
		result.setProperty("bar", 23);
		return result;
	}

	QObject* getChild() { return child_.get(); }
	boost::shared_ptr<Exposable> getChild2() { return child_; }
	Exposable* getChild3() { return child_.get(); }
	boost::shared_ptr<QObject> getChild4() { return child_; }
	//QSharedPointer<QObject> getChild5() { return QSharedPointer<QObject>(getChild()); }
};




#endif /* EXPOSABLE_H_ */
