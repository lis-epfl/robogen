/*
 * @(#) RobogenUtils.h   1.0   Feb 17, 2013
 *
 * Basil Huber (basil.huber@epfl.ch)
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
#ifndef PARSING_UTILS_H_
#define PARSING_UTILS_H_

#include <sstream>


namespace robogen
{
	/**
	 * Parse the string to get the numeric value.
	 * Supports NAN, INF, E and e (exponential).
	 * @param str input string
	 * @return parsed float; if failed, returns 0.0f
	 */
	static inline float parse_float(const std::string& str)
	{

		float f = 0.0f;
		std::istringstream istr(str);

		
		istr >> f;

		if(istr.fail())
		{
			// check for NAN, nan or NaN
			if (!str.compare(0, 3, "NAN") || !str.compare(0, 3, "nan") || !str.compare(0, 3, "NaN"))
			{
				f = NAN;
			}
			else if (!str.compare(0, 3, "INF") || !str.compare(0, 3, "inf") || !str.compare(0, 3, "Inf"))
			{
				f = std::numeric_limits<float>::infinity();
			}
		}

		return f;
	}

	/**
	 * Parse the string to get the numeric value.
	 * Supports NAN, INF, E and e (exponential).
	 * @param str input string
	 * @return parsed double; if failed, returns 0.0
	 */
	static inline double parse_double(const std::string& str)
	{

		double d = 0.0;
		std::istringstream istr(str);

		istr >> d;

		if(istr.fail())
		{
			// check for NAN, nan or NaN
			if (!str.compare(0, 3, "NAN") || !str.compare(0, 3, "nan") || !str.compare(0, 3, "NaN"))
			{
				d = NAN;
			}
			else if (!str.compare(0, 3, "INF") || !str.compare(0, 3, "inf") || !str.compare(0, 3, "Inf"))
			{
				d = std::numeric_limits<double>::infinity();
			}
		}

		return d;
	}
}
#endif /* PARSING_UTILS_H_ */