/*
 * @(#) IMU.h  1.0   Dec 15, 2014
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
 * \file IMU.h
 * 
 * \author Gregoire Heitz
 *   
 * \brief Useful functions for imu
 *
 ******************************************************************************/


#ifndef IMU_H_
#define IMU_H_ 

#include "Arduino.h"
#include "MPU6050.h"

class IMU
{
  public:  
    IMU();
    
    
    /**
     * \brief 		update the imu
     * 
     * \param 	imu 	Pointer to the imu struct
     * \param 	accelGyro 	Pointer to the MPU6050
     */
    void update(MPU6050* accelGyro);
    
    float scaledAccel[3];
    float scaledGyro[3];
    bool initialized;
    int counter;
    int16_t rawAccel[3];
    int16_t rawGyro[3];
    
    int16_t gyroOffset[3];

};



#endif /* IMU_H_ */
