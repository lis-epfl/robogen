/*
 * @(#) PartList.h   1.0   Nov 5, 2013
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

#ifndef ROBOGEN_PARTLIST_H_
#define ROBOGEN_PARTLIST_H_

#include <map>
#include <vector>
#include <string>

namespace robogen {

#define PART_TYPE_ACTIVE_CARDAN 	"ActiveCardan"
#define PART_TYPE_ACTIVE_HINGE 		"ActiveHinge"
#define PART_TYPE_ACTIVE_WHEEL 		"ActiveWheel"
#define PART_TYPE_ACTIVE_WHEG 		"ActiveWheg"
#define PART_TYPE_CORE_COMPONENT 	"CoreComponent"
#define PART_TYPE_FIXED_BRICK 		"FixedBrick"
#define PART_TYPE_LIGHT_SENSOR 		"LightSensor"
#define PART_TYPE_PARAM_JOINT 		"ParametricJoint"
#define PART_TYPE_PASSIVE_CARDAN 	"PassiveCardan"
#define PART_TYPE_PASSIVE_HINGE 	"PassiveHinge"
#define PART_TYPE_PASSIVE_WHEEL 	"PassiveWheel"
#define PART_TYPE_ROTATOR 			"Rotator"
#define PART_TYPE_TOUCH_SENSOR 		"TouchSensor"

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

} /* namespace robogen */

#endif /* ROBOGEN_PARTLIST_H_ */
