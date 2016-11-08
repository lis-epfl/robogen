/*
 * @(#) Grammar.h   1.0   Nov 8, 2016
 *
 * Carlos Malanche (carlos.malancheflores@epfl.ch)
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

#ifndef GRAMMAR_H
#define GRAMMAR_H

#if 0 // set to 1 to use fake robots for evolution algorithm benchmark
#include "evolution/representation/FakeRobotRepresentation.h"
#else

#include <string>
#include <set>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "config/RobogenConfig.h"
#include "evolution/representation/PartRepresentation.h"
#include "evolution/representation/NeuralNetworkRepresentation.h"
#include "utils/network/TcpSocket.h"
#include "robogen.pb.h"

class Grammar{
public:
    Grammar();
private:
    int x;
};

#endif

#endif //GRAMMAR_H