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

#include <boost/shared_ptr.hpp>
#include <QCoreApplication>

Q_DECLARE_SMART_POINTER_METATYPE(boost::shared_ptr)

class Exposable : public QObject {
	Q_OBJECT
	int a, b;
	boost::shared_ptr<Exposable> child;

 public:
	Exposable() : a(5), b(7), child(new Exposable(2, 3)) {}
	Exposable(int a, int b) : a(a), b(b) {}
	Exposable(const Exposable& exposable) :
		QObject(),
		a(exposable.a), b(exposable.b), child(exposable.child) {
	}
	~Exposable() {}


 public slots:
	int getA() { return a; }
	int getB() { return b; }
	QObject* getChild() { return child.get(); }
	boost::shared_ptr<Exposable> getChild2() { return child; }
	Exposable* getChild3() { return child.get(); }
	boost::shared_ptr<QObject> getChild4() { return child; }
};

Q_DECLARE_METATYPE(Exposable*)

#endif /* EXPOSABLE_H_ */
