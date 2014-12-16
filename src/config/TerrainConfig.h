/*
 * @(#) TerrainConfig.h   1.0   Mar 12, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_TERRAIN_CONFIG_H_
#define ROBOGEN_TERRAIN_CONFIG_H_

#include <string>

namespace robogen {

/**
 * Terrain configuration parameters
 */
class TerrainConfig {

public:

	/**
	 * Initializes a flat terrain
	 *
	 * @param length
	 * @param width
	 */
	TerrainConfig(float length, float width, float friction) :
			flat_(true), length_(length), width_(width), height_(0),
			friction_(friction) {

	}

	/**
	 * Initializes a rough terrain
	 *
	 * @param heightFieldFileName
	 * @param length
	 * @param width
	 * @param height
	 */
	TerrainConfig(const std::string& heightFieldFileName, float length,
			float width, float height, float friction) :
				flat_(false), heightFieldFileName_(heightFieldFileName),
				length_(length), width_(width), height_(height),
				friction_(friction) {

	}

	/**
	 * Destructor
	 */
	virtual ~TerrainConfig() {

	}

	/**
	 * @return true if the terrain is flat, false otherwise
	 */
	bool isFlat() {
		return flat_;
	}

	/**
	 * @return the heightfield file name
	 */
	const std::string& getHeightFieldFileName() const {
		return heightFieldFileName_;
	}

	/**
	 * @return the length of the terrain
	 */
	float getLength() {
		return length_;
	}

	/**
	 * @return the width of the terrain
	 */
	float getWidth() {
		return width_;
	}

	/**
	 * @return the height of the terrain
	 */
	float getHeight() {
		return height_;
	}

	/**
	 * @return the friction coefficient of the terrain
	 * TODO: make this be more configurable than just global
	 * 	friction coefficient
	 */
	float getFriction() {
		return friction_;
	}

private:

	/**
	 * True if the terrain is flat, false if rough
	 */
	bool flat_;

	/**
	 * If terrain is rugged, contains the height field file name
	 */
	std::string heightFieldFileName_;

	/**
	 * Terrain length
	 */
	float length_;

	/**
	 * Terrain width
	 */
	float width_;

	/**
	 * Terrain maximum height
	 */
	float height_;

	/**
	 * Terrain friction coefficient
	 */
	float friction_;

};

}

#endif /* ROBOGEN_TERRAIN_CONFIG_H_ */
