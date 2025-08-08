/*
 * I2CServo.cpp
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
#include <I2CServo.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/types.h>
#include <time.h>

#define MOTOR_ADDR 0x40
#define MOTOR_FREQ 100

#define SUBADR1            0x02
#define SUBADR2            0x03
#define SUBADR3            0x04
#define MODE1              0x00
#define PRESCALE           0xFE
#define LED0_ON_L          0x06
#define LED0_ON_H          0x07
#define LED0_OFF_L         0x08
#define LED0_OFF_H         0x09
#define ALLLED_ON_L        0xFA
#define ALLLED_ON_H        0xFB
#define ALLLED_OFF_L       0xFC
#define ALLLED_OFF_H       0xFD

#define MAX_VALUE 4096

#define DEV_NUM 1

using namespace std;

I2CServo::I2CServo(I2C* i2c) {
    int tmp;
    
    this->i2c = i2c;
    
    this->i2c->set_addr(MOTOR_ADDR);

    tmp = i2c_smbus_write_byte_data(this->i2c->file, MODE1, 0x00);
    if (tmp < 0) {
        cout << "Error writing servo mode: " << tmp << endl;
        exit(1);
    }

    setFreq(MOTOR_FREQ);

    for (int i = 0; i < NUM_SERVO_CHANNELS; i++) {
        this->microsValues[i] = -1;
    }
}

void I2CServo::setFreq(float freq) {
    float prescaleval;
    __u8 prescale;
    int tmp;
    __u8 oldmode, newmode;
    struct timespec sleeptime;
  
    prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    prescale = (int) (prescaleval+0.5);

    this->i2c->set_addr(MOTOR_ADDR);
  
    //cout << "Freq: " << freq << endl;
    //cout << "Prescale est: " << prescaleval << endl;
    //cout << "Prescale: " << ((int)prescale) << endl;
    
    tmp = i2c_smbus_read_byte_data(this->i2c->file, MODE1);
    if (tmp < 0) {
        cout << "Error reading mode: " << tmp << endl;
        exit(1);
    }
    oldmode = (__u8) tmp;
    newmode = (oldmode & 0x7F) | 0x10;
    tmp = i2c_smbus_write_byte_data(this->i2c->file, MODE1, newmode);
    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        exit(1);
    }
    
    tmp = i2c_smbus_write_byte_data(this->i2c->file, PRESCALE, prescale);
    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        exit(1);
    }
    
    tmp = i2c_smbus_write_byte_data(this->i2c->file, MODE1, oldmode);
    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        exit(1);
    }
    
    sleeptime.tv_sec = 0;
    sleeptime.tv_nsec = 5000000;
    nanosleep(&sleeptime, NULL);
    
    tmp = i2c_smbus_write_byte_data(this->i2c->file, MODE1, oldmode | 0x80);
    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        exit(1);
    }
    
    this->offPerMicro = MAX_VALUE / (1000000.0/freq);
}

void I2CServo::setMicros(int channel, int micros) {
    int tmp;
    int on = 0;
    int off = (int) (micros * this->offPerMicro);

    if (this->microsValues[channel] == micros) {
        // early return for servio micros already set
        return;
    }
    
    //cout << "Write channel " << channel << " on=" << on << " off=" << off << endl; 

    this->i2c->set_addr(MOTOR_ADDR);
    
    tmp = i2c_smbus_write_byte_data(this->i2c->file, LED0_ON_L + 4*channel, on & 0xFF);
    if (tmp < 0) {
        cout << "Error writing ON_L: " << tmp << endl;
        exit(1);
    }
    
    tmp = i2c_smbus_write_byte_data(this->i2c->file, LED0_ON_H + 4*channel, on >> 8);
    if (tmp < 0) {
        cout << "Error writing ON_H: " << tmp << endl;
        exit(1);
    }
    
    tmp = i2c_smbus_write_byte_data(this->i2c->file, LED0_OFF_L + 4*channel, off & 0xFF);
    if (tmp < 0) {
        cout << "Error writing OFF_L: " << tmp << endl;
        exit(1);
    }
    
    tmp = i2c_smbus_write_byte_data(this->i2c->file, LED0_OFF_H + 4*channel, off >> 8);
    if (tmp < 0) {
        cout << "Error writing OFF_H: " << tmp << endl;
        exit(1);
    }

    this->microsValues[channel] = micros;
}
