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
#include "RCoreClient.h"
#include "I2C.h"
#include "I2CServo.h"
#include "I2CIMU.h"
#include "I2CLidar.h"
#include "DrogonPosition.h" 
#include "DrogonController.h"

#include <iostream>
#include <chrono>         // std::chrono::seconds

class DrogonFlight {
public:
    DrogonFlight();
    ~DrogonFlight();

    void run();
    void run_debug();

    double to_seconds(std::chrono::high_resolution_clock::time_point now_tp);
    double duration_to_milliseconds(std::chrono::high_resolution_clock::duration now_tp);

    void display_imu(double t);
    void log_imu(double t);
    void log_pid(double t);

private:
    void read_imu();
    void read_rcore(double t);

    void motors_arm();
    void motors_disarm();

    void control_update(double t);
    void update_motors();

    FILE* imu_f;
    FILE* pid_f;

    void print_vec(FILE* f, vector3d* vec);

    DrogonPosition pos;
    DrogonController ctrl;

    vector3d accelValues;
    vector3d gyroValues;
    vector3d magValues;
    int lidarValue;

    bool armed;
    bool controlEngaged;

    int motorValues[4];
    double motorAdjusts[4];
    double zRotAdjust;

    double motorMaster;
    double motorRotate[3];

    RCoreClient rcore;

    I2C i2c;
    
    I2CLSM303Accel accel;
    I2CLSM303Mag mag;
    I2CL3GD20Gyro gyro;

    I2CServo motors;

    I2CLidar lidar;
};


#endif  // __DROGON_FLIGHT_H__