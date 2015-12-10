/*
 * @(#) Sensor.h   1.0   Feb 25, 2013
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
#ifndef ROBOGEN_SENSOR_H_
#define ROBOGEN_SENSOR_H_

#include <string>

namespace robogen {

class Sensor {

public:


	inline Sensor(std::string label) : label_(label), value_(0) {

	}

	const std::string &getLabel(){
		return label_;
	}

	virtual ~Sensor () {

	}

	void updateValue(double value) {
		value_ = value;
	}

	double read() {
		return value_;
	}

private:

	double value_;

	const std::string label_;
};

}


#endif /* ROBOGEN_SENSOR_H_ */
