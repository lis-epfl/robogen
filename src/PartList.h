/*
 * @(#) PartList.h   1.0   Nov 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Andrea Maesani, Joshua Auerbach
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

#ifndef ROBOGEN_PARTLIST_H_
#define ROBOGEN_PARTLIST_H_

#include <map>
#include <vector>
#include <string>

#define ALLOW_ROTATIONAL_COMPONENTS
//#define ALLOW_CARDANS

#define ENFORCE_PLANAR

#define IR_SENSORS_ENABLED
#ifndef IR_SENSORS_ENABLED
	#define TOUCH_SENSORS_ENABLED
#endif


namespace robogen {

#ifdef ALLOW_CARDANS
	#define PART_TYPE_ACTIVE_CARDAN 	"ActiveCardan"
#endif

#define PART_TYPE_ACTIVE_HINGE 		"ActiveHinge"

#ifdef ALLOW_ROTATIONAL_COMPONENTS
	#define PART_TYPE_ACTIVE_WHEEL 		"ActiveWheel"
	#define PART_TYPE_ACTIVE_WHEG 		"ActiveWheg"
#endif

#define PART_TYPE_CORE_COMPONENT 	"CoreComponent"
#define PART_TYPE_CORE_COMPONENT_NO_IMU 	"CoreComponentNoIMU"
#define PART_TYPE_FIXED_BRICK 		"FixedBrick"
#define PART_TYPE_LIGHT_SENSOR 		"LightSensor"
#define PART_TYPE_PARAM_JOINT 		"ParametricJoint"

#ifdef ALLOW_CARDANS
	#define PART_TYPE_PASSIVE_CARDAN 	"PassiveCardan"
#endif

#define PART_TYPE_PASSIVE_HINGE 	"PassiveHinge"
#ifdef ALLOW_ROTATIONAL_COMPONENTS
	#define PART_TYPE_PASSIVE_WHEEL 	"PassiveWheel"
	#define PART_TYPE_ROTATOR 			"Rotator"
#endif

#ifdef IR_SENSORS_ENABLED
	#define PART_TYPE_IR_SENSOR			"IrSensor"
#endif

#ifdef TOUCH_SENSORS_ENABLED
	#define PART_TYPE_TOUCH_SENSOR 		"TouchSensor"
#endif

extern const std::map<char, std::string> LEGACY_PART_TYPE_MAP;
extern const std::map<char, std::string> PART_TYPE_MAP;
extern const std::map<std::string, char> INVERSE_PART_TYPE_MAP;
extern const std::map<std::string, unsigned int> PART_TYPE_ARITY_MAP;
extern const std::map<std::string, unsigned int> PART_TYPE_PARAM_COUNT_MAP;
extern const std::map<std::pair<std::string, unsigned int>,
	std::pair<double, double> > PART_TYPE_PARAM_RANGE_MAP;
extern const std::map<std::string, std::vector<std::string> >
	PART_TYPE_MOTORS_MAP;
extern const std::map<std::string, std::vector<std::string> >
	PART_TYPE_SENSORS_MAP;

bool isCore(char partType);
bool isCore(std::string partType);

} /* namespace robogen */

#endif /* ROBOGEN_PARTLIST_H_ */
