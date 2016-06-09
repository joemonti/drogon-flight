/*
 * DrogonFlight.h
 *
 * This file is part of Drogon.
 *
 * Drogon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Drogon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Drogon.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Joseph Monti <joe.monti@gmail.com>
 * Copyright (c) 2013 Joseph Monti All Rights Reserved, http://joemonti.org/
 */

#ifndef __DROGON_FLIGHT_H__
#define __DROGON_FLIGHT_H__

#include "DrogonCommon.h"
#include "I2C.h"
#include "I2CIMU.h"
#include "DrogonPosition.h" 
#include "DrogonController.h"

#include <iostream>


class DrogonFlight {
public:
    DrogonFlight(void);

    void run(void);

    void read_imu(void);

    void close(void);

    double now(void);

private:
    void print_vec(FILE* f, vector3d* vec);

    //I2C i2c;
    
    //I2CLSM303Accel accel(&i2c);
    //I2CLSM303Mag mag(&i2c);
    //I2CL3GD20Gyro gyro(&i2c);

    DrogonPosition pos;
    DrogonController ctrl;

    vector3d accelValues;
    vector3d gyroValues;
    vector3d magValues;

    int motorValues[4];
    double motorAdjusts[4];
    double zRotAdjust;

    double motorMaster;
    double motorRotate[3];
};


#endif  // __DROGON_FLIGHT_H__