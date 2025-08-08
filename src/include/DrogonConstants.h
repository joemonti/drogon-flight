/*
 * DrogonConstants.h
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

#ifndef __DROGON_CONSTANTS_H__
#define __DROGON_CONSTANTS_H__

#include <math.h>

#ifndef PI
#define PI M_PI
#endif

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#define constrain(x,a,b) min(b,max(a,x))

#define INIT_KP 0.01
#define INIT_KI 0.01
#define INIT_KD 0.001

#define MAX_ERR_TOTAL 20.0

#define ARM_LENGTH 280
#define ARM_ANGLE_A (45.0 * PI / 180.0)
#define ARM_ANGLE_B ((90.0+45.0) * PI / 180.0)

#define ACCEL_SCALE 0.1276

#define ACCEL_VAR_SQ_A 0.4
#define ACCEL_VAR_SQ_B 12.0
                    // 3.0

#define GYRO_VAR_SQ_A 0.4
#define GYRO_VAR_SQ_B 2.0

#define ACCEL_MERGE_VAR_SQ 2.0
#define GYRO_MERGE_VAR_SQ 1.5

#define POS_VAR_SQ_A 0.8
#define POS_VAR_SQ_B 0.4
#define POS_VAR_SQ_V 0.6

#define VEL_VAR_SQ_A 0.8
#define VEL_VAR_SQ_B 0.2

#define Z_ROT_VAR_SQ 0.25
#define Z_ROT_UPDATE_VAR_SQ 0.5

#define TUNER_INIT 2.75
#define TUNER_TOLERANCE 0.00001
#define TUNER_SCALE_MINOR 1.05
#define TUNER_SCALE_MAJOR 1.2
#define TUNER_SCALE_DOWN 0.8

const int MIN_MOTOR_VALUE = 1000;
const int MAX_MOTOR_VALUE = 2000;
const double MAX_MOTOR_ADJUST = ( MAX_MOTOR_VALUE - MIN_MOTOR_VALUE ) * 0.25;
const double MIN_MOTOR_ADJUST = -MAX_MOTOR_ADJUST;
const double MAX_MOTOR_ZROT_ADJUST = ( MAX_MOTOR_VALUE - MIN_MOTOR_VALUE ) * 0.15;
const double MIN_MOTOR_ZROT_ADJUST = -MAX_MOTOR_ZROT_ADJUST;
const double MOTOR_DIFF_SCALE = 0.1;

const int CONTROL_ENGAGE_THRESHOLD_HIGH = MIN_MOTOR_VALUE + (int) ( ( MAX_MOTOR_VALUE - MIN_MOTOR_VALUE ) * 0.1 );
const int CONTROL_ENGAGE_THRESHOLD_LOW = MIN_MOTOR_VALUE + (int) ( ( MAX_MOTOR_VALUE - MIN_MOTOR_VALUE ) * 0.08 );

#endif
