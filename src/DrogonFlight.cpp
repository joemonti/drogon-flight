/*
 * DrogonFlight.cpp
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

#include "DrogonFlight.h"

#include <iostream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

using namespace std;


DrogonFlight::DrogonFlight(void) : ctrl(&pos)
{
    accelValues.x = 0;
    accelValues.y = 0;
    accelValues.z = 0;

    magValues.x = 0;
    magValues.y = 0;
    magValues.z = 0;

    gyroValues.x = 0;
    gyroValues.y = 0;
    gyroValues.z = 0;
    
    motorMaster = 0.0;
    motorRotate[0] = 0.0;
    motorRotate[1] = 0.0;
    motorRotate[2] = 0.0;
}

void DrogonFlight::run(void)
{
    FILE* f;
    char fname[100];
    int i;

    double t = now();

    sprintf(fname, "imu.%ld.log", (long)(t*1000.0));
    f = fopen(fname, "w");

    for (i = 0; i < 100; i++) {
        t = now();

        read_imu();
        pos.update(t, &accelValues, &gyroValues);

        fprintf(f, "%f", t);

        //accel.read(&vec);
        //print_vec(f, &vec);
        
        //mag.read(&vec);
        //print_vec(f, &vec);

        //gyro.read(&vec);
        //print_vec(f, &vec);

        fprintf(f, "\n");

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    fclose(f);
}

void DrogonFlight::read_imu(void)
{
    //accel.read(&accelValues);

    //mag.read(&magValues);

    //gyro.read(&gyroValues);
}

void DrogonFlight::close(void)
{
    //i2c.close();
}


void DrogonFlight::print_vec(FILE* f, vector3d* vec)
{
    fprintf(f, ",%f,%f,%f", vec->x, vec->y, vec->z);
}

double DrogonFlight::now(void)
{
    chrono::high_resolution_clock::time_point now_tp = chrono::high_resolution_clock::now();
    return now_tp.time_since_epoch().count() * chrono::high_resolution_clock::period::num / static_cast<double>(chrono::high_resolution_clock::period::den);
}