/*
 * @(#) IMU.c  1.0   Dec 15, 2014
 *
 * Grégoire Heitz (gregoire.heitz@epfl.ch),
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Grégoire Heitz, Joshua Auerbach
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

/*******************************************************************************
 * \file IMU.c
 * 
 * \author Gregoire Heitz
 *   
 * \brief Useful functions for imu
 *
 ******************************************************************************/

#include "IMU.h"

#define LPF_GYRO (0.2f)
#define LPF_ACC (0.2f) 

#define ACCEL_SCALE_FACTOR (9.81f/2100.0f)
#define GYRO_SCALE_FACTOR ((1.0f/131.0f) * (PI/180.0f)) * 1.5  // 1.5 to scale up to match simulation, since amplitutde is reduced by LPF

IMU::IMU()
{
    initialized = false;
    counter = 0;
}
  

void IMU::update(MPU6050* accelGyro)
{
  accelGyro->getMotion6(&this->rawAccel[0], &this->rawAccel[1], &this->rawAccel[2], 
                      &this->rawGyro[0], &this->rawGyro[1], &this->rawGyro[2]);
  
  for(int i=0; i<3; i++) {
    if (this->initialized) {
      this->scaledGyro[i]   = (1-LPF_GYRO)  * this->scaledGyro[i]      + LPF_GYRO * (this->rawGyro[i] + this->gyroOffset[i]) * GYRO_SCALE_FACTOR;
      this->scaledAccel[i]  = (1-LPF_ACC)   * this->scaledAccel[i]     + LPF_ACC * this->rawAccel[i] * ACCEL_SCALE_FACTOR;
    } else {
      this->scaledGyro[i]     = (this->rawGyro[i] + this->gyroOffset[i]) * GYRO_SCALE_FACTOR;
      this->scaledAccel[i]     = this->rawAccel[i] * ACCEL_SCALE_FACTOR;
    }
  }
  
  if(!this->initialized && ((this->counter++)>10))
    this->initialized = true;
}

