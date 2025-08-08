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

#include <iostream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <iomanip>
#include <thread>         // std::this_thread::sleep_for
#include <vector>
#include <cmath>
#include <numeric>

#include "DrogonFlight.h"
#include "DrogonConstants.h"

using namespace std;


// Global flag to indicate if the header has been printed
// This is a simple way to manage state for demonstration.
// In a larger application, you might pass a state object or use a class.
bool header_printed = false;

double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to calculate the mean
double calculateMean(const std::vector<double>& data) {
    if (data.empty()) {
        return 0.0; // Handle empty list case
    }
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
}

// Function to calculate the standard deviation
double calculateStandardDeviation(const std::vector<double>& data) {
    if (data.empty()) {
        return 0.0; // Handle empty list case
    }

    double mean = calculateMean(data);
    double sumSquaredDifferences = 0.0;

    for (double value : data) {
        sumSquaredDifferences += std::pow(value - mean, 2);
    }

    // For population standard deviation:
    double variance = sumSquaredDifferences / data.size(); 

    return std::sqrt(variance);
}

DrogonFlight::DrogonFlight() : ctrl(&pos), rcore("localhost"), accel(&i2c), mag(&i2c), gyro(&i2c), motors(&i2c), lidar(&i2c)
{
    zero_vector3d(&accelValues);
    zero_vector3d(&magValues);
    zero_vector3d(&gyroValues);
    lidarValue = -1;

    armed = false;
    controlEngaged = false;
    
    motorMaster = 0.0;
    motorRotate[0] = 0.0;
    motorRotate[1] = 0.0;
    motorRotate[2] = 0.0;

    char fname[100];

    chrono::high_resolution_clock::time_point now_tp = chrono::high_resolution_clock::now();
    double t = to_seconds(now_tp);

    sprintf(fname, "logs/imu.%ld.log", (long)t);
    imu_f = fopen(fname, "w");

    sprintf(fname, "logs/pid.%ld.log", (long)t);
    pid_f = fopen(fname, "w");
}

DrogonFlight::~DrogonFlight()
{
    motors_disarm();
    rcore.close();
    i2c.close();
    fclose(imu_f);
    fclose(pid_f);
}

void DrogonFlight::run()
{
    chrono::high_resolution_clock::time_point now_tp = chrono::high_resolution_clock::now();
    chrono::high_resolution_clock::time_point end_tp = chrono::high_resolution_clock::now();
    chrono::high_resolution_clock::time_point last_tp = chrono::high_resolution_clock::now();
    chrono::milliseconds log_interval(100);
    chrono::milliseconds update_interval(20);
    chrono::high_resolution_clock::duration sleep_time;
    chrono::high_resolution_clock::duration process_time;
    chrono::milliseconds min_update_interval(2);

    double t = 0; //to_seconds(now_tp);

    while (true) {
        now_tp = chrono::high_resolution_clock::now();
        t = to_seconds(now_tp);

        read_rcore(t);

        read_imu();
        pos.update(t, &accelValues, &gyroValues);

        if (armed) {
            control_update(t);
            update_motors();
        }
        
        if ( (now_tp - last_tp) > log_interval ) {
            log_imu(t);

            last_tp = chrono::high_resolution_clock::now();
        }

        end_tp = chrono::high_resolution_clock::now();
        
        process_time = (end_tp - now_tp);
        sleep_time = update_interval - process_time;

        if ( sleep_time < min_update_interval ) {
            sleep_time = min_update_interval;
            std::cout << "WARNING: Reached minimum sleep time (" << duration_to_milliseconds(sleep_time) << ")" << std::endl;
        }

        this_thread::sleep_for(sleep_time);
    }
}

void DrogonFlight::run_debug()
{
    chrono::high_resolution_clock::time_point start_tp = chrono::high_resolution_clock::now();
    chrono::high_resolution_clock::time_point now_tp = chrono::high_resolution_clock::now();
    chrono::high_resolution_clock::time_point end_tp = chrono::high_resolution_clock::now();
    chrono::high_resolution_clock::time_point last_tp = chrono::high_resolution_clock::now();
    chrono::milliseconds log_interval(500);
    chrono::milliseconds update_interval(20);
    chrono::seconds post_arm_sleep(5);
    chrono::seconds motor_run_time(30);
    chrono::high_resolution_clock::duration sleep_time;
    chrono::high_resolution_clock::duration process_time;
    chrono::milliseconds min_update_interval(2);

    std::vector<double> accelXValues;
    std::vector<double> accelYValues;
    std::vector<double> gyroXValues;
    std::vector<double> gyroYValues;

    size_t initial_values_size = 1200;
    accelXValues.reserve(initial_values_size);
    accelYValues.reserve(initial_values_size);
    gyroXValues.reserve(initial_values_size);
    gyroYValues.reserve(initial_values_size);

    double MOTOR_SPEED = 30.0;

    double t = 0; //to_seconds(now_tp);

    this->motors_arm();

    std::cout << "Motors Armed" << std::endl;

    this_thread::sleep_for(post_arm_sleep);

    int target = (int) map_double( MOTOR_SPEED, 0.0, 100.0, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
    target = constrain( target, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
    motorValues[0] = 
        motorValues[1] = 
        motorValues[2] = 
        motorValues[3] = 
        motorValues[4] = target;
    update_motors();

    std::cout << "Motors set: " << target << std::endl;

    this_thread::sleep_for(post_arm_sleep);

    while ((now_tp - start_tp) < motor_run_time) {
        now_tp = chrono::high_resolution_clock::now();
        t = to_seconds(now_tp);

        read_rcore(t);

        read_imu();
        pos.update(t, &accelValues, &gyroValues);

        accelXValues.push_back(accelValues.x);
        accelYValues.push_back(accelValues.y);
        gyroXValues.push_back(gyroValues.x);
        gyroYValues.push_back(gyroValues.y);

        control_update(t);
        
        if ( (now_tp - last_tp) > log_interval ) {
            display_imu(t);
            log_imu(t);

            last_tp = chrono::high_resolution_clock::now();
        }

        end_tp = chrono::high_resolution_clock::now();
        
        process_time = (end_tp - now_tp);
        sleep_time = update_interval - process_time;

        if ( sleep_time < min_update_interval ) {
            sleep_time = min_update_interval;
            std::cout << "WARNING: Reached minimum sleep time (" << duration_to_milliseconds(sleep_time) << ")" << std::endl;
        }

        this_thread::sleep_for(sleep_time);
    }

    this->motors_disarm();
    std::cout << std::endl << std::endl;

    std::cout << "Accel X StdDev: " << calculateStandardDeviation(accelXValues) << std::endl;
    std::cout << "Accel Y StdDev: " << calculateStandardDeviation(accelYValues) << std::endl;
    std::cout << "Gyro X StdDev: " << calculateStandardDeviation(gyroXValues) << std::endl;
    std::cout << "Gyro Y StdDev: " << calculateStandardDeviation(gyroYValues) << std::endl;
}

void DrogonFlight::read_imu()
{
    accel.read(&accelValues);
    mag.read(&magValues);
    gyro.read(&gyroValues);
    lidarValue = lidar.read();
}

void DrogonFlight::read_rcore(double t)
{
    if ( rcore.read() ) {
        if ( rcore.is_arm_data() ) {
            uint8_t arm = rcore.get_arm_data();
            if (arm <= 0) {
                motors_disarm();
            } else {
                motors_arm();
            }
            printf("%f,ARM,%d\n", t, arm);
        } else if ( rcore.is_motor_data() ) {
            double motor = rcore.get_motor_data();
            motorMaster = motor;
            printf("%f,MOTOR,%.4f\n", t, motor);
        } else if ( rcore.is_pid_data() ) {
            vector3d* pidData = rcore.get_pid_data();
            ctrl.pidA.set_thetas(pidData->x, pidData->y, pidData->z);
            ctrl.pidB.set_thetas(pidData->x, pidData->y, pidData->z);
            printf("%f,PID,%.4f,%.4f,%.4f\n", t, pidData->x, pidData->y, pidData->z);
            free(pidData);
        }
    }
}

void DrogonFlight::motors_arm()
{
    if (!armed) {
        motorValues[0] = 
            motorValues[1] = 
            motorValues[2] = 
            motorValues[3] = 
            motorValues[4] = MIN_MOTOR_VALUE;
        update_motors();
        armed = true;
    }
}

void DrogonFlight::motors_disarm()
{
    if (armed) {
        motorValues[0] = 
            motorValues[1] = 
            motorValues[2] = 
            motorValues[3] = 
            motorValues[4] = 0;
        update_motors();
        armed = false;
    }
}

void DrogonFlight::control_update(double t)
{
    int target = (int) map_double( motorMaster, 0.0, 100.0, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
    target = constrain( target, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );

    if (controlEngaged) {
        if ( target < CONTROL_ENGAGE_THRESHOLD_LOW ) {
            //ctrl.tune();
            //log_pid( t );
            ctrl.reset( t );
            //nextTuneTime = millis() + TUNER_FREQUENCY;

            motorAdjusts[0] = 0.0;
            motorAdjusts[1] = 0.0;
            motorAdjusts[2] = 0.0;
            motorAdjusts[3] = 0.0;

            zRotAdjust = 0.0;

            controlEngaged = false;
        } else {
            /*
            if ( millis() > nextTuneTime ) {
                ctrl.tune();
                log_pid( );
                nextTuneTime = millis() + TUNER_FREQUENCY;
            }
            */

            ctrl.control_update( t, motorRotate );

            motorAdjusts[0] = ctrl.motorAdjusts[0];
            motorAdjusts[1] = ctrl.motorAdjusts[1];
            motorAdjusts[2] = ctrl.motorAdjusts[2];
            motorAdjusts[3] = ctrl.motorAdjusts[3];

            zRotAdjust = ctrl.zRotAdjust;
        }
    } else if ( target >= CONTROL_ENGAGE_THRESHOLD_HIGH ) {
        ctrl.reset( t );
        //nextTuneTime = millis() + TUNER_FREQUENCY;

        ctrl.control_update( t, motorRotate );

        motorAdjusts[0] = ctrl.motorAdjusts[0];
        motorAdjusts[1] = ctrl.motorAdjusts[1];
        motorAdjusts[2] = ctrl.motorAdjusts[2];
        motorAdjusts[3] = ctrl.motorAdjusts[3];

        zRotAdjust = ctrl.zRotAdjust;

        controlEngaged = true;
    }

    motorAdjusts[0] = constrain( motorAdjusts[0], MIN_MOTOR_ADJUST, MAX_MOTOR_ADJUST );
    motorAdjusts[1] = constrain( motorAdjusts[1], MIN_MOTOR_ADJUST, MAX_MOTOR_ADJUST );
    motorAdjusts[2] = constrain( motorAdjusts[2], MIN_MOTOR_ADJUST, MAX_MOTOR_ADJUST );
    motorAdjusts[3] = constrain( motorAdjusts[3], MIN_MOTOR_ADJUST, MAX_MOTOR_ADJUST );

    zRotAdjust = max( MIN_MOTOR_ZROT_ADJUST, min( MAX_MOTOR_ZROT_ADJUST, zRotAdjust ) );

    motorValues[0] = constrain( target + motorAdjusts[0] + zRotAdjust, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
    motorValues[1] = constrain( target + motorAdjusts[1] + zRotAdjust, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
    motorValues[2] = constrain( target + motorAdjusts[2] + zRotAdjust, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
    motorValues[3] = constrain( target + motorAdjusts[3] + zRotAdjust, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
}

void DrogonFlight::update_motors() {
  motors.setMicros( 0, motorValues[0] );
  motors.setMicros( 1, motorValues[1] );
  motors.setMicros( 2, motorValues[2] );
  motors.setMicros( 3, motorValues[3] );
}

void DrogonFlight::display_imu(double t)
{
    // If the header hasn't been printed yet, print it.
    // This ensures the column names appear only once at the top.
    if (!header_printed) {
        std::cout << std::setw(15) << "Time"
                  << std::setw(10) << "AccX"
                  << std::setw(10) << "AccY"
                  << std::setw(10) << "GyrX"
                  << std::setw(10) << "GyrY"
                  << std::setw(10) << "PosX"
                  << std::setw(10) << "PosY"
                  << std::setw(7)  << "Mtr0"
                  << std::setw(7)  << "Mtr1"
                  << std::setw(7)  << "Mtr2"
                  << std::setw(7)  << "Mtr3"
                  << std::setw(7)  << "Lidar"
                  << std::endl; // Use std::endl to move to the next line after the header
        header_printed = true;
    }

    // Move cursor to the beginning of the line using '\r'.
    // This will overwrite the previous line of data.
    // We print spaces to ensure any leftover characters from a previous, potentially
    // longer line (though not an issue with fixed-width) are cleared.
    // However, with fixed-width, simply overwriting is usually sufficient.
    // The length of the line is calculated based on the setw values.
    // (10*7) + (7*5) = 70 + 35 = 105 characters + 10 for time + spaces between columns
    // A safe bet is to print a string of spaces equal to the max expected line length.
    // For this fixed-width output, the line length is constant, so just '\r' is enough.
    std::cout << "\r";

    // Set formatting for floating-point numbers: fixed decimal notation, 3 decimal places.
    std::cout << std::fixed << std::setprecision(3);

    // Print the data with specified fixed widths for each column.
    std::cout << std::setw(15) << t
              << std::setw(10) << accelValues.x
              << std::setw(10) << accelValues.y
              << std::setw(10) << gyroValues.x
              << std::setw(10) << gyroValues.y
              << std::setw(10) << pos.position.x
              << std::setw(10) << pos.position.y;

    // Set width for integer values.
    std::cout << std::setw(7)  << motorValues[0]
              << std::setw(7)  << motorValues[1]
              << std::setw(7)  << motorValues[2]
              << std::setw(7)  << motorValues[3]
              << std::setw(7)  << lidarValue;

    // Flush the output buffer immediately to ensure the update is visible.
    // Without flushing, output might be buffered and only appear later.
    std::cout.flush();
}

void DrogonFlight::log_imu(double t)
{
    fprintf(imu_f, "%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d\n", 
        t, 
        accelValues.x, accelValues.y, 
        gyroValues.x, gyroValues.y, 
        pos.position.x, pos.position.y,
        motorValues[0], motorValues[1], motorValues[2], motorValues[3],
        lidarValue);
    fflush(imu_f);
    //rcore.send_log(t, accelValues, gyroValues, pos.position, motorValues);
}

void DrogonFlight::log_pid(double t)
{
    double* thetas = ctrl.pidA.get_thetas();
    double* adjusts = ctrl.pidATuner.get_adjusts();

    fprintf(pid_f, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
        t, 
        ctrl.pidATuner.get_last_error(),
        ctrl.pidATuner.get_best_error(),
        adjusts[0], adjusts[1], adjusts[2],
        thetas[0], thetas[1], thetas[2]);
    fflush(pid_f);
}


void DrogonFlight::print_vec(FILE* f, vector3d* vec)
{
    fprintf(f, ",%f,%f,%f", vec->x, vec->y, vec->z);
}

double DrogonFlight::to_seconds(chrono::high_resolution_clock::time_point now_tp)
{
    return now_tp.time_since_epoch().count() * chrono::high_resolution_clock::period::num / static_cast<double>(chrono::high_resolution_clock::period::den);
}

double DrogonFlight::duration_to_milliseconds(chrono::high_resolution_clock::duration now_tp)
{
    return now_tp.count() * 1000.0 * chrono::high_resolution_clock::period::num / static_cast<double>(chrono::high_resolution_clock::period::den);
}