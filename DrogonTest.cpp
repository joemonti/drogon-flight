/*
 * DrogonTest.cpp
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

#include "arduino/DrogonController.h"

#include <iostream>
using namespace std;

#include <sys/time.h>

DrogonPosition position;
DrogonController dc(&position);

double zeroPosition[] = { 0.0, 0.0, 0.0 };

extern "C" {
	void set_position( double x, double y ) {
		position.x = x;
		position.y = y;
	}
	void position_update( long micros, double accelValues[3], double gyroValues[3] ) {
		position.update( micros, accelValues, gyroValues );
	}
	double get_position_x(void) {
		return position.x;
	}
	double get_position_y(void) {
		return position.y;
	}
	void control_update(unsigned long micros) {
		dc.control_update(micros, zeroPosition);
	}
	void update_thetas( double kp, double ki, double kd ) {
		dc.pidAbsoluteA.set_thetas( kp, ki, kd );
	}
	double get_motor_adjust(int idx) {	
		return dc.motorAdjusts[idx];
	}
	double get_pid_a_error_p(void) {
		return dc.pidAbsoluteA.ep;
	}
    double get_pid_a_error_i(void) {
        return dc.pidAbsoluteA.ei;
    }
    double get_pid_a_error_d(void) {
        return dc.pidAbsoluteA.ed;
    }
    double get_pid_b_error_p(void) {
        return dc.pidAbsoluteB.ep;
    }
    double get_pid_b_error_i(void) {
        return dc.pidAbsoluteB.ei;
    }
    double get_pid_b_error_d(void) {
        return dc.pidAbsoluteB.ed;
    }
	double get_err_a(void) {
		return dc.pidAbsoluteA.error;
	}
	double get_err_b(void) {
		return dc.pidAbsoluteB.error;
	}
	double get_motor_offset_a(void) {
		return dc.motorOffsetA;
	}
	double get_motor_offset_b(void) {
		return dc.motorOffsetB;
	}

	long micros() {
		struct timeval tv;

		gettimeofday( &tv, NULL );
		
		return 1000000 * tv.tv_sec + tv.tv_usec;
	}
}


int main(int argc, char** argv) {
	
	long m = micros();
	
	//DrogonPosition pos;
	//DrogonController dc(&pos);
	
	cout << "Hello Word : " << m << " micros\n";
}
