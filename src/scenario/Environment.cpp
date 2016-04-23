/*
 * @(#) Environment.cpp   1.0   Dec 10, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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

#include "Environment.h"


namespace robogen {

Environment::Environment(dWorldID odeWorld, dSpaceID odeSpace,
		boost::shared_ptr<RobogenConfig> robogenConfig) :
				odeWorld_(odeWorld), odeSpace_(odeSpace),
				robogenConfig_(robogenConfig),
				timeElapsed_(0),
				ambientLight_(DEFAULT_AMBIENT_LIGHT) {
}

Environment::~Environment() {
	odeWorld_ = 0;
	odeSpace_ = 0;
	terrain_.reset();
	obstacles_.clear();
}

bool Environment::init() {
	// Setup terrain
	boost::shared_ptr<TerrainConfig> terrainConfig =
			robogenConfig_->getTerrainConfig();

	terrain_ = boost::shared_ptr<Terrain>(
			new Terrain(odeWorld_, odeSpace_));
	if (terrainConfig->getType() == TerrainConfig::FLAT) {
		if(!terrain_->initFlat(terrainConfig->getLength(),
				terrainConfig->getWidth())) {
			return false;
		}
	}
#ifndef DISABLE_HEIGHT_MAP
	else if (terrainConfig->getType() == TerrainConfig::ROUGH) {
		if(!terrain_->initRough(terrainConfig->getHeightFieldFileName(),
				terrainConfig->getLength(), terrainConfig->getWidth(),
				terrainConfig->getHeight())) {
			return false;
		}
	}
#endif

	return true;
}




}

