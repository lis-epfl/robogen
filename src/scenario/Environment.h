/*
 * @(#) environment.h   1.0   Mar 6, 2013
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
#ifndef ROBOGEN_ENVIRONMENT_H_
#define ROBOGEN_ENVIRONMENT_H_

#include "model/objects/LightSource.h"

namespace robogen {

class Environment {

public:

	Environment() : timeElapsed_(0) {

	}

	virtual ~Environment() {

	}

	void setTimeElapsed(float timeElapsed) {
		timeElapsed_ = timeElapsed;
	}

	float getTimeElapsed() {
		return timeElapsed_;
	}

	void setLightSources(
			const std::vector<boost::shared_ptr<LightSource> >& lightSources) {
		lightSources_ = lightSources;
	}

	const std::vector<boost::shared_ptr<LightSource> >& getLightSources() {
		return lightSources_;
	}

	void setAmbientLight(float ambientLight) {
		ambientLight_ = ambientLight;
	}

	float getAmbientLight() {
		return ambientLight_;
	}

private:

	/**
	 * Time elapsed since the last "refresh" of the environment
	 */
	float timeElapsed_;

	/**
	 * Light sources in the environment
	 */
	std::vector<boost::shared_ptr<LightSource> > lightSources_;

	/**
	 * Ambient light
	 */
	float ambientLight_;

};

}

#endif /* ROBOGEN_ENVIRONMENT_H_ */
