/*
 * DrogonPosition.cpp
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

#include "DrogonPosition.h"

DrogonPosition::DrogonPosition(void) {
	x = 0.0;	
	y = 0.0;
	
	velocityX = 0.0;
	velocityY = 0.0;
	
	varSqX = INIT_VAR_SQ;
	varSqY = INIT_VAR_SQ;
	
	sensorVarSq = calc_var( ACCEL_VAR_SQ, GYRO_VAR_SQ );
	
	velocityVarSq = INIT_VEL_VAR_SQ;
	
	lastMicros = 0;
}

void DrogonPosition::update( long micros, const double accelValues[3], const double gyroValues[3] ) {
	double accelX = (-accelValues[1]*ACCEL_SCALE);
	double accelY = (accelValues[0]*ACCEL_SCALE);
	
	double gyroX = x + gyroValues[0];
	double gyroY = y + gyroValues[1];
	
	double lastX = x;
	double lastY = y;
	
	double sensorX = calc_mean( accelX, ACCEL_VAR_SQ, gyroX, GYRO_VAR_SQ );
	double sensorY = calc_mean( accelY, ACCEL_VAR_SQ, gyroY, GYRO_VAR_SQ );
	
	double elapsedSeconds = 0.0;
	
	if ( lastMicros > 0 ) {
		elapsedSeconds = ( micros - lastMicros ) / 1000000.0;
	}
	
	if ( elapsedSeconds > 0.0 ) {
		x += ( velocityX * elapsedSeconds );
		y += ( velocityY * elapsedSeconds );
	}
	
	x = calc_mean( x, varSqX, sensorX, sensorVarSq );
	y = calc_mean( y, varSqY, sensorY, sensorVarSq );
	
	varSqX = calc_var( varSqX, sensorVarSq );
	varSqY = calc_var( varSqY, sensorVarSq );
	
	varSqX *= VAR_UPDATE_SCALE;
	varSqY *= VAR_UPDATE_SCALE;
	
	if ( elapsedSeconds > 0.0 ) {
		velocityX = calc_mean( velocityX, velocityVarSq, ( x - lastX ) / elapsedSeconds, VEL_VAR_SQ );
		velocityY = calc_mean( velocityX, velocityVarSq, ( y - lastY ) / elapsedSeconds, VEL_VAR_SQ );
		
		velocityVarSq = calc_var( velocityVarSq, VEL_VAR_SQ );
		
		velocityVarSq *= VAR_UPDATE_SCALE;
	}
	
	lastMicros = micros;
}

double DrogonPosition::calc_mean( double mean1, double var1, double mean2, double var2 ) {
	return ( var2 * mean1 + var1 * mean2 ) / ( var1 + var2 );
}

double DrogonPosition::calc_var( double var1, double var2 ) {
	return 1 / ( ( 1 / var1 ) + ( 1 / var2 ) );
}

