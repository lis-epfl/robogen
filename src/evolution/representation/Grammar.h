/*
 * @(#) Grammar.h   1.0   Nov 8, 2016
 *
 * Carlos Malanche (carlos.malancheflores@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2016 Carlos Malanche
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
#include <boost/random/bernoulli_distribution.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_01.hpp>
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
            typedef std::map< std::string, std::string > effectMap;
            struct buildStep{
                std::string parentPartId;
                unsigned int parentPartSlot;
                boost::shared_ptr<PartRepresentation> newPart;
                unsigned int newPartSlot;
                unsigned int motorNeuronType;
            };

            /**
             * Type defined to simplify the insertion and deletion of nodes
             */
            typedef bool (SubRobotRepresentation::*RuleOperation)(boost::shared_ptr<SubRobotRepresentation>&);
            
            /**
             * Produces a random rule with the given successor and predecessor
             * @param iterations number of times the rule is applied
             * @param predecessor SubRobot of the pattern to search
             * @param successor SubRobot of the replace pattern
             */
            Rule(int iterations, boost::shared_ptr<SubRobotRepresentation> predecessor,
                boost::random::mt19937 &rng, boost::shared_ptr<EvolverConfiguration> conf);

            /**
             * Check if the predecessor is identical to the candidate
             * @param candidate PartRepresentation, a node in the robot to be compared.a
             */
            bool matchesPredecessor(boost::shared_ptr<PartRepresentation> candidate);

            /**
             * Apply the rule for one single iteration in the node from a robot
             * @param candidate PartRepresentation, a node in the robot to be compared.a
             */
            bool applyRule(boost::shared_ptr<SubRobotRepresentation> robot, boost::shared_ptr<PartRepresentation> node);

            /**
             * Get the number of times the rule should be applied.
             */
            int getNumIterations(void);

            /**
             * Return the predecessor of the Rule
             */
            boost::shared_ptr<SubRobotRepresentation> getPredecessor(void);

            /**
             * Return the successor of the Rule
             */
            boost::shared_ptr<SubRobotRepresentation> getSuccessor(void);
        private:

            /*The killer Rule: includes or not the root element of the successor.
            */
            bool rootRule;

            /**
             * Map from the intact predecessor
             */
            boost::shared_ptr<effectMap> deleteMap_;

            /**
             * Map from the final successor
             */
            boost::shared_ptr<effectMap> buildMap_;

            /**
             * Number of iterations for the rules.
             */
            int iterations_;
            
            /**
             * Predecessor of the rule
             */
            boost::shared_ptr<SubRobotRepresentation> predecessor_;
            
            /**
             * Deletion steps (IDs) from the predecessor
             */
            std::vector< std::string > deletions_;

            /**
             * Insertions steps from the predecessor
             */
            std::vector<buildStep> insertions_;

            /**
             * Successor of the rule
             */
            boost::shared_ptr<SubRobotRepresentation> successor_;
        };

        /**
         * Default constructor, which takes as an axiom an initially random robot.
         * @param axiom shared pointer to a subrobotrepresentation which will become the axiom.
         */
        Grammar(boost::shared_ptr<SubRobotRepresentation> axiom);

        /**
         * Get a shared pointer to the axiom
         */
        boost::shared_ptr<SubRobotRepresentation> getAxiom(void);

        /**
         * Grow a tree according to the current rules and alphabet of the grammar.
         */
        boost::shared_ptr<SubRobotRepresentation> buildTree(void);

        int getNumberOfRules(void);

        boost::shared_ptr<Rule> getRule(int id);

        bool addRule(boost::shared_ptr<Rule> newRule);
        void popLastRule(void);
        bool lastBuildFailed();
    private:
        /**
         * Axiom of the grammar
         */
        boost::shared_ptr<SubRobotRepresentation> axiom_;
        
        /**
         * Vector of rules in the grammar
         */
        std::vector< boost::shared_ptr<Rule> > rules_;

        bool lastBuildWorked;
    };
}

#endif //GRAMMAR_H