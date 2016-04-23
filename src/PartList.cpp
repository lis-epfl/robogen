/*
 * @(#) PartList.cpp   1.0   Nov 5, 2013
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
#include "PartList.h"

namespace robogen {

//first define init functions that will populate these maps

std::map<char, std::string> initLegacyPartTypeMap() {
	std::map<char, std::string> partTypeMap;
#ifdef ALLOW_CARDANS
	partTypeMap['K'] = PART_TYPE_ACTIVE_CARDAN;
	partTypeMap['C'] = PART_TYPE_PASSIVE_CARDAN;
#endif
#ifdef ALLOW_ROTATIONAL_COMPONENTS
	partTypeMap['J'] = PART_TYPE_ACTIVE_WHEEL;
	partTypeMap['G'] = PART_TYPE_ACTIVE_WHEG;
	partTypeMap['W'] = PART_TYPE_PASSIVE_WHEEL;
	partTypeMap['R'] = PART_TYPE_ROTATOR;
#endif
	return partTypeMap;
}

std::map<char, std::string> initPartTypeMap(const std::map<char, std::string>
											&legacyPartTypeMap) {
	std::map<char, std::string> partTypeMap;
	partTypeMap.insert(legacyPartTypeMap.begin(), legacyPartTypeMap.end());

	partTypeMap['I'] = PART_TYPE_ACTIVE_HINGE;

	partTypeMap['E'] = PART_TYPE_CORE_COMPONENT;
	partTypeMap['N'] = PART_TYPE_CORE_COMPONENT_NO_IMU;
	partTypeMap['F'] = PART_TYPE_FIXED_BRICK;
	partTypeMap['L'] = PART_TYPE_LIGHT_SENSOR;
	partTypeMap['B'] = PART_TYPE_PARAM_JOINT;

	partTypeMap['H'] = PART_TYPE_PASSIVE_HINGE;
#ifdef IR_SENSORS_ENABLED
	partTypeMap['D'] = PART_TYPE_IR_SENSOR;
#endif
#ifdef TOUCH_SENSORS_ENABLED
	partTypeMap['T'] = PART_TYPE_TOUCH_SENSOR;
#endif
	return partTypeMap;
}

std::map<std::string, unsigned int> initPartTypeArityMap() {
	std::map<std::string, unsigned int> partTypeArityMap;
#ifdef ALLOW_CARDANS
	partTypeArityMap[PART_TYPE_ACTIVE_CARDAN] = 1;
#endif
	partTypeArityMap[PART_TYPE_ACTIVE_HINGE] = 1;
#ifdef ALLOW_ROTATIONAL_COMPONENTS
	partTypeArityMap[PART_TYPE_ACTIVE_WHEEL] = 0;
	partTypeArityMap[PART_TYPE_ACTIVE_WHEG] = 0;
#endif
#ifdef ENFORCE_PLANAR
	partTypeArityMap[PART_TYPE_CORE_COMPONENT] = 4;
	partTypeArityMap[PART_TYPE_CORE_COMPONENT_NO_IMU] = 4;
	partTypeArityMap[PART_TYPE_FIXED_BRICK] = 3;
#else
	partTypeArityMap[PART_TYPE_CORE_COMPONENT] = 6;
	partTypeArityMap[PART_TYPE_CORE_COMPONENT_NO_IMU] = 6;
	partTypeArityMap[PART_TYPE_FIXED_BRICK] = 5;
#endif
	partTypeArityMap[PART_TYPE_LIGHT_SENSOR] = 0;
	partTypeArityMap[PART_TYPE_PARAM_JOINT] = 1;
#ifdef ALLOW_CARDANS
	partTypeArityMap[PART_TYPE_PASSIVE_CARDAN] = 1;
#endif
	partTypeArityMap[PART_TYPE_PASSIVE_HINGE] = 1;
#ifdef ALLOW_ROTATIONAL_COMPONENTS
	partTypeArityMap[PART_TYPE_PASSIVE_WHEEL] = 1;
	partTypeArityMap[PART_TYPE_ROTATOR] = 1;
#endif
#ifdef IR_SENSORS_ENABLED
	partTypeArityMap[PART_TYPE_IR_SENSOR] = 0;
#endif
#ifdef TOUCH_SENSORS_ENABLED
	partTypeArityMap[PART_TYPE_TOUCH_SENSOR] = 0;
#endif
	return partTypeArityMap;
}

std::map<std::string, unsigned int> initPartTypeParamCountMap() {
	std::map<std::string, unsigned int> partTypeParamCountMap;
#ifdef ALLOW_CARDANS
	partTypeParamCountMap[PART_TYPE_ACTIVE_CARDAN] = 0;
#endif
	partTypeParamCountMap[PART_TYPE_ACTIVE_HINGE] = 0;
#ifdef ALLOW_ROTATIONAL_COMPONENTS
	partTypeParamCountMap[PART_TYPE_ACTIVE_WHEEL] = 1;
	partTypeParamCountMap[PART_TYPE_ACTIVE_WHEG] = 1;
#endif
	partTypeParamCountMap[PART_TYPE_CORE_COMPONENT] = 0;
	partTypeParamCountMap[PART_TYPE_CORE_COMPONENT_NO_IMU] = 0;
	partTypeParamCountMap[PART_TYPE_FIXED_BRICK] = 0;
	partTypeParamCountMap[PART_TYPE_LIGHT_SENSOR] = 0;
	partTypeParamCountMap[PART_TYPE_PARAM_JOINT] = 3;
#ifdef ALLOW_CARDANS
	partTypeParamCountMap[PART_TYPE_PASSIVE_CARDAN] = 0;
#endif
	partTypeParamCountMap[PART_TYPE_PASSIVE_HINGE] = 0;
#ifdef ALLOW_ROTATIONAL_COMPONENTS
	partTypeParamCountMap[PART_TYPE_PASSIVE_WHEEL] = 1;
	partTypeParamCountMap[PART_TYPE_ROTATOR] = 0;
#endif
#ifdef IR_SENSORS_ENABLED
	partTypeParamCountMap[PART_TYPE_IR_SENSOR] = 0;
#endif
#ifdef TOUCH_SENSORS_ENABLED
	partTypeParamCountMap[PART_TYPE_TOUCH_SENSOR] = 0;
#endif
	return partTypeParamCountMap;
}

std::map<std::pair<std::string, unsigned int>, std::pair<double, double> >
												initPartTypeParamRangeMap() {
	std::map<std::pair<std::string, unsigned int>,
		std::pair<double, double> > partTypeParamRangeMap;
#ifdef ALLOW_ROTATIONAL_COMPONENTS
	partTypeParamRangeMap[std::make_pair(PART_TYPE_ACTIVE_WHEEL, 0)] =
			std::make_pair(0.03, 0.08); // radius in m   --- TODO update wiki
	partTypeParamRangeMap[std::make_pair(PART_TYPE_ACTIVE_WHEG, 0)] =
			std::make_pair(0.03, 0.08); // radius in m
#endif
	partTypeParamRangeMap[std::make_pair(PART_TYPE_PARAM_JOINT, 0)] =
			std::make_pair(0.02, 0.1); // length in m
	partTypeParamRangeMap[std::make_pair(PART_TYPE_PARAM_JOINT, 1)] =
			std::make_pair(-90.0, 90.0); // tilt (alpha) in degrees
	partTypeParamRangeMap[std::make_pair(PART_TYPE_PARAM_JOINT, 2)] =
#ifdef ENFORCE_PLANAR
			std::make_pair(0.0, 0.0); // rotation (beta) in degrees
#else
			std::make_pair(0.0, 180.0); // rotation (beta) in degrees
#endif
#ifdef ALLOW_ROTATIONAL_COMPONENTS
	partTypeParamRangeMap[std::make_pair(PART_TYPE_PASSIVE_WHEEL, 0)] =
			std::make_pair(0.03, 0.08); // radius in m
#endif
	return partTypeParamRangeMap;
}

std::map<std::string, std::vector<std::string> > initPartTypeMotorsMap() {
	std::map<std::string, std::vector<std::string> > partTypeMotorsMap;
#ifdef ALLOW_CARDANS
	{
		std::vector<std::string> motors;
		motors.push_back(PART_TYPE_ACTIVE_CARDAN + std::string("-tilt-1"));
		motors.push_back(PART_TYPE_ACTIVE_CARDAN + std::string("-tilt-2"));
		partTypeMotorsMap[PART_TYPE_ACTIVE_CARDAN] = motors;
	}
#endif
	{
#ifdef ALLOW_ROTATIONAL_COMPONENTS
		std::string singleMotorParts[] = { PART_TYPE_ACTIVE_HINGE,
				PART_TYPE_ACTIVE_WHEEL, PART_TYPE_ACTIVE_WHEG,
				PART_TYPE_ROTATOR };
		int numSingleMotorParts = 4;
#else
		std::string singleMotorParts[] = { PART_TYPE_ACTIVE_HINGE };
		int numSingleMotorParts = 1;
#endif
		for (int i = 0; i < numSingleMotorParts; i++) {
			std::vector<std::string> motors;
			motors.push_back(singleMotorParts[i]);
			partTypeMotorsMap[singleMotorParts[i]] = motors;
		}
	}

	// need to insert empty vectors for all others
	for (std::map<char, std::string>::const_iterator  it =
			PART_TYPE_MAP.begin(); it != PART_TYPE_MAP.end(); ++it) {
		if(partTypeMotorsMap.count(it->second) == 0) {
			partTypeMotorsMap[it->second] = std::vector<std::string>(0);
		}
	}

	return partTypeMotorsMap;
}
std::map<std::string, std::vector<std::string> > initPartTypeSensorsMap() {
	std::map<std::string, std::vector<std::string> > partTypeSensorsMap;
	{
		std::vector<std::string> sensors;
		sensors.push_back("x-acceleration");
		sensors.push_back("y-acceleration");
		sensors.push_back("z-acceleration");
		sensors.push_back("Pitch");
		sensors.push_back("Roll");
		sensors.push_back("Yaw");
		partTypeSensorsMap[PART_TYPE_CORE_COMPONENT] = sensors;
	}

	{
		std::vector<std::string> sensors;
		sensors.push_back(PART_TYPE_LIGHT_SENSOR);
		partTypeSensorsMap[PART_TYPE_LIGHT_SENSOR] = sensors;
	}

#ifdef IR_SENSORS_ENABLED
	{
		std::vector<std::string> sensors;
		sensors.push_back(PART_TYPE_IR_SENSOR);
		partTypeSensorsMap[PART_TYPE_IR_SENSOR] = sensors;
	}
#endif
#ifdef TOUCH_SENSORS_ENABLED
	{
		std::vector<std::string> sensors;
		sensors.push_back(PART_TYPE_TOUCH_SENSOR + std::string("-left"));
		sensors.push_back(PART_TYPE_TOUCH_SENSOR + std::string("-right"));
		partTypeSensorsMap[PART_TYPE_TOUCH_SENSOR] = sensors;
	}
#endif
	// need to insert empty vectors for all others
	for (std::map<char, std::string>::const_iterator  it =
			PART_TYPE_MAP.begin(); it != PART_TYPE_MAP.end(); ++it) {
		if(partTypeSensorsMap.count(it->second) == 0) {
			partTypeSensorsMap[it->second] = std::vector<std::string>(0);
		}
	}

	return partTypeSensorsMap;
}

template<typename _OrigKey, typename _OrigValue>
std::map<_OrigValue, _OrigKey> inverseMap(
		std::map<_OrigKey, _OrigValue> origMap) {

	std::map<_OrigValue, _OrigKey> inverseMap;
	for (typename std::map<_OrigKey, _OrigValue>::iterator iterator =
			origMap.begin(); iterator != origMap.end(); iterator++) {
		inverseMap[iterator->second] = iterator->first;
	}

	return inverseMap;
}

bool isCore(char partType) {
	return isCore(PART_TYPE_MAP.at(partType));

}

bool isCore(std::string partType) {
	return ((partType.compare(PART_TYPE_CORE_COMPONENT) == 0)
			||
			(partType.compare(PART_TYPE_CORE_COMPONENT_NO_IMU) == 0));
}

//initialize the maps
const std::map<char, std::string> LEGACY_PART_TYPE_MAP =
		initLegacyPartTypeMap();
const std::map<char, std::string> PART_TYPE_MAP =
		initPartTypeMap(LEGACY_PART_TYPE_MAP);
const std::map<std::string, char> INVERSE_PART_TYPE_MAP =
		inverseMap(PART_TYPE_MAP);
const std::map<std::string, unsigned int> PART_TYPE_ARITY_MAP =
		initPartTypeArityMap();
const std::map<std::string, unsigned int> PART_TYPE_PARAM_COUNT_MAP =
		initPartTypeParamCountMap();
const std::map<std::pair<std::string, unsigned int>,
		std::pair<double, double> >
		PART_TYPE_PARAM_RANGE_MAP = initPartTypeParamRangeMap();
const std::map<std::string, std::vector<std::string> > PART_TYPE_MOTORS_MAP =
		initPartTypeMotorsMap();
const std::map<std::string, std::vector<std::string> > PART_TYPE_SENSORS_MAP =
		initPartTypeSensorsMap();

} /* namespace robogen */

