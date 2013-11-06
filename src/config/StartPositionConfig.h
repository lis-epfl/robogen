/*
 * @(#) StartPositionConfig.h   1.0   Mar 12, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
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
#ifndef ROBOGEN_START_POSITION_CONFIG_H_
#define ROBOGEN_START_POSITION_CONFIG_H_

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "config/StartPosition.h"
#include "robogen.pb.h"

namespace robogen {

/**
 * Contains the list of allowed starting positions for the robots
 */
class StartPositionConfig {

public:

	/**
	 * Starting positions and azimuths
	 */
	StartPositionConfig(
			std::vector<boost::shared_ptr<StartPosition> > startPosition) :
		startPosition_(startPosition){

	}

	/**
	 * @return the starting positions
	 */
	std::vector<boost::shared_ptr<StartPosition> > getStartPosition() {
		return startPosition_;
	}

	/**
	 * @return the starting position at index i
	 */
	boost::shared_ptr<StartPosition> getStartPosition(int i) {
		if (i < 0 || i >= (int) startPosition_.size()) {
			std::cout << "The starting position " << i <<
					" is not specified in the configuration file." << std::endl;
			return boost::shared_ptr<StartPosition>();
		}
		return startPosition_[i];
	}

	/**
	 * Serialize starting positions into a SimulatorConf message
	 */
	void serialize(robogenMessage::SimulatorConf &message){
		for (unsigned int i=0; i<startPosition_.size(); ++i){
			robogenMessage::StartPosition *curr = message.add_startpositions();
			curr->set_azimuth(startPosition_[i]->getAzimuth());
			curr->set_x(startPosition_[i]->getPosition().x());
			curr->set_y(startPosition_[i]->getPosition().y());
		}
	}

private:

	std::vector<boost::shared_ptr<StartPosition> > startPosition_;
};

}


#endif /* ROBOGEN_START_POSITION_CONFIG_H_ */
