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

#include <string>
#include <set>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <queue>

#include "config/EvolverConfiguration.h"

#include "config/RobogenConfig.h"
#include "PartList.h"
#include "evolution/representation/PartRepresentation.h"
#include "evolution/representation/NeuralNetworkRepresentation.h"
#include "evolution/representation/SubRobotRepresentation.h"
#include "utils/network/TcpSocket.h"
#include "robogen.pb.h"

namespace robogen{

    class Grammar{
    public:
        class Rule{
        public:
            Rule(int iterations, boost::shared_ptr<PartRepresentation> predecessor,
                            boost::shared_ptr<PartRepresentation> successor);

            bool matchesPredecessor(boost::shared_ptr<PartRepresentation> candidate);
            void ignoreChildrenCount(bool state);
            void ignorePosition(bool state);
            void ignoreOrientation(bool state);
            int getNumIterations(void);
        private:
            int iterations_;
            bool ignoreChildrenCount_;
            bool ignorePosition_;
            bool ignoreOrientation_;
            boost::shared_ptr<PartRepresentation> predecessor_;
            boost::shared_ptr<PartRepresentation> successor_;
        };

        /**
        * Default constructor, which takes as an axiom an initially random robot.
        */
        Grammar(boost::shared_ptr<SubRobotRepresentation> r);

        /**
        * Grow a tree according to the current rules and alphabet of the grammar.
        */
        boost::shared_ptr<SubRobotRepresentation> buildTree(void);
    private:
        boost::shared_ptr<SubRobotRepresentation> axiom_;
        std::vector< boost::shared_ptr<Rule> > rules_;
    };
}

#endif //GRAMMAR_H