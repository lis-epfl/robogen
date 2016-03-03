/*
 * @(#) ArduinoNNConfiguration.h   1.0   Sep 10, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * based on previous work by:
 * Gregoire Heitz (gregoire.heitz@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014
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

#ifndef ARDUINONNCONFIGURATION_H_
#define ARDUINONNCONFIGURATION_H_

#include <string>

#define MAX_DIGITAL_PINS 12
#define MAX_ANALOG_PINS 4
#define MAX_PWM_PINS 4


namespace robogen {

namespace arduino {

/**
 * Slots for digital signals: touch sensors and servos
 */
const std::string digitalOrder[] = {"D9", "D10", "D5", "D6", "D11", "D13",
		"ROLL", "PITCH", "YAW", "AUX1", "D7", "D4"};

/**
 * Slots for analog signals: light sensors, can also be used as digital
 */

const std::string analogOrder[] = {"A0", "A1", "A2", "A3"};

/**
 * Slots for PWM signals: rotation motors (some are duplicate of digital)
 */
const std::string pwmOrder[] = {"D11", "D5", "D6", "D13"};

/**
 * Integer codes for input type tab
 */
enum inputType{
	LIGHT_SENSOR,
	TOUCH_SENSOR,
	IMU,
	IR_SENSOR
};

/**
 * Integer codes for motor type
 */
enum motorType{
	POSITION_CONTROL,
	VELOCITY_CONTROL
};


} /* namespace arduino */
} /* namespace robogen */
#endif /* ARDUINONNCONFIGURATION_H_ */
