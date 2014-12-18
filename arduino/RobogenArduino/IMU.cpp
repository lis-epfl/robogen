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

void imuUpdate(IMU* imu, MPU6050* accelGyro)
{
  accelGyro->getMotion6(&imu->rawAccel[0], &imu->rawAccel[1], &imu->rawAccel[2], 
                      &imu->rawGyro[0], &imu->rawGyro[1], &imu->rawGyro[2]);
  
  for(int i=0; i<3; i++) {
    if (imu->initialized) {
      imu->scaledGyro[i]   = (1-LPF_GYRO)  * imu->scaledGyro[i]      + LPF_GYRO * (imu->rawGyro[i] + imu->gyroOffset[i]) * GYRO_SCALE_FACTOR;
      imu->scaledAccel[i]  = (1-LPF_ACC)   * imu->scaledAccel[i]     + LPF_ACC * imu->rawAccel[i] * ACCEL_SCALE_FACTOR;
    } else {
      imu->scaledGyro[i]     = (imu->rawGyro[i] + imu->gyroOffset[i]) * GYRO_SCALE_FACTOR;
      imu->scaledAccel[i]     = imu->rawAccel[i] * ACCEL_SCALE_FACTOR;
    }
  }
  
  if(!imu->initialized && ((imu->counter++)>10))
    imu->initialized = true;
}

