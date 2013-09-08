/*
 * @(#) Replacer.h   1.0   Sep 8, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
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

#ifndef REPLACER_H_
#define REPLACER_H_

#include "evolution/engine/Population.h"

namespace robogen {

/**
 * Implements Replacement in the evolution process.
 */
class Replacer {
public:
	/**
	 * Creates a replacer that replaces the n worst individuals of the old
	 * population with the n best of the new one.
	 */
	Replacer(unsigned int nReplace);

	virtual ~Replacer();

	/**
	 * Perform replacement on two evaluated populations.
	 * @todo const previous, but non-const orderedEvaluatedRobots prevents
	 */
	void replace(Population *current, Population *previous);

private:
	/**
	 * Amplitude of replacement
	 */
	unsigned int nReplace_;
};

} /* namespace robogen */
#endif /* REPLACER_H_ */
