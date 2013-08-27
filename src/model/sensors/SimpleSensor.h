/*
 * @(#) SimpleSensor.h   1.0   Mar 6, 2013
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
#ifndef ROBOGEN_SIMPLE_SENSOR_H_
#define ROBOGEN_SIMPLE_SENSOR_H_

#include <string>
#include "model/sensors/Sensor.h"

namespace robogen {

class SimpleSensor : public Sensor {

public:

	SimpleSensor(std::string label) : label_(label) {

	}

	virtual std::string &getLabel(){
		return label_;
	}

	virtual ~SimpleSensor () {

	}

	void update(float value) {
		value_ = value;
	}

	float read() {
		return value_;
	}

private:

	float value_;

	std::string label_;
};

}


#endif /* ROBOGEN_SIMPLE_SENSOR_H_ */
