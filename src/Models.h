/*
 * @(#) Models.h   1.0   March 4, 2013
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
#ifndef ROBOGEN_MODELS_H_
#define ROBOGEN_MODELS_H_

#include "model/motors/ServoMotor.h"
#include "model/motors/RotationMotor.h"
#include "model/objects/LightSource.h"
#include "model/sensors/ImuSensor.h"
#include "model/sensors/LightSensor.h"
#include "model/sensors/TouchSensor.h"
#include "model/sensors/IrSensor.h"

#include "model/components/actuated/ActiveCardanModel.h"
#include "model/components/actuated/ActiveHingeModel.h"
#include "model/components/actuated/ActiveWheelModel.h"
#include "model/components/actuated/ActiveWhegModel.h"
#include "model/components/actuated/RotateJointModel.h"
#include "model/components/CardanModel.h"
#include "model/components/HingeModel.h"
#include "model/components/ParametricBrickModel.h"
#include "model/components/PassiveWheelModel.h"

#include "model/components/perceptive/CoreComponentModel.h"
#include "model/components/perceptive/LightSensorModel.h"
#include "model/components/perceptive/TouchSensorModel.h"
#include "model/components/perceptive/IrSensorModel.h"

#endif /* ROBOGEN_MODELS_H_ */
