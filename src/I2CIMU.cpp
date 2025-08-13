/*
 * I2CIMU.cpp
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
#include <I2CIMU.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#ifdef __cplusplus
}
#endif

using namespace std;

I2CLSM303Accel::I2CLSM303Accel(I2C* i2c)
{
    int tmp;
    
    this->i2c = i2c;
    
    i2c->set_addr(LSM303_ADDRESS_ACCEL);
    
    tmp = i2c_smbus_write_byte_data(i2c->file, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        exit(1);
    }
}

void I2CLSM303Accel::read(vector3d* data)
{
    int tmp;
    __u8 buf[6];
    
    this->i2c->set_addr(LSM303_ADDRESS_ACCEL);
    
    tmp = i2c_smbus_read_i2c_block_data(this->i2c->file, LSM303_REGISTER_ACCEL_OUT_X_L_A|0x80, 6, (__u8*) buf);
    if (tmp != 6) {
        cout << "Error reading accel data: " << tmp << endl;
        exit(1);
    }
    
    data->x = ((int16_t)( (buf[3] << 8) | buf[2] )) / 16.0;
    data->y = -((int16_t)( (buf[1] << 8) | buf[0] )) / 16.0;
    data->z = ((int16_t)( (buf[5] << 8) | buf[4] )) / 16.0;
}

I2CLSM303Mag::I2CLSM303Mag(I2C* i2c)
{
    int tmp;
    
    this->i2c = i2c;
    
    i2c->set_addr(LSM303_ADDRESS_MAG);
    
    tmp = i2c_smbus_write_byte_data(i2c->file, LSM303_REGISTER_MAG_MR_REG_M, 0x00);
    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        exit(1);
    }
    
    tmp = i2c_smbus_read_byte_data(i2c->file, LSM303_REGISTER_MAG_CRA_REG_M);
    if (tmp < 0) {
        cout << "Error reading mode: " << tmp << endl;
        exit(1);
    }
    if (((uint8_t)tmp) != 0x10) {
        cout << "Invalid CRA_REG: " << tmp << endl;
        exit(1);
    }
    
    set_gain(LSM303_MAGGAIN_1_3);
}

void I2CLSM303Mag::set_gain(lsm303MagGain gain)
{
    int tmp;
    
    this->i2c->set_addr(LSM303_ADDRESS_MAG);
    
    tmp = i2c_smbus_write_byte_data(this->i2c->file, LSM303_REGISTER_MAG_CRB_REG_M, (__u8)gain);
    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        exit(1);
    }
}

void I2CLSM303Mag::read(vector3d* data)
{
    int tmp;
    __u8 buf[6];
    
    this->i2c->set_addr(LSM303_ADDRESS_MAG);
    
    tmp = i2c_smbus_read_i2c_block_data(this->i2c->file, LSM303_REGISTER_MAG_OUT_X_H_M, 6, (__u8*) buf);
    if (tmp != 6) {
        cout << "Error reading mag data: " << tmp << endl;
        exit(1);
    }
    
    data->x = (int16_t)( buf[1] | ((int16_t)buf[0] << 8) );
    data->y = (int16_t)( buf[5] | ((int16_t)buf[4] << 8) );
    data->z = (int16_t)( buf[3] | ((int16_t)buf[2] << 8) );
}


I2CL3GD20Gyro::I2CL3GD20Gyro(I2C *i2c) 
{
    int tmp;
    
    this->i2c = i2c;
    
    i2c->set_addr(L3GD20_ADDRESS);

    tmp = i2c_smbus_read_byte_data(i2c->file, L3GD20_REGISTER_WHO_AM_I);
    if (tmp < 0) {
        cout << "Error reading mode: " << tmp << endl;
        exit(1);
    }
    if ((((uint8_t)tmp) != L3GD20_ID) && (((uint8_t)tmp) != L3GD20H_ID)) {
        cout << "Invalid id: " << tmp << endl;
        exit(1);
    }

    tmp = i2c_smbus_write_byte_data(this->i2c->file, L3GD20_REGISTER_CTRL_REG1, 0x0F);
    if (tmp < 0) {
        cout << "Error reading mode: " << tmp << endl;
        exit(1);
    }

    this->set_range(L3DS20_RANGE_250DPS);
}

void I2CL3GD20Gyro::set_range(l3gd20Range_t range)
{
    int tmp = -1;

    this->i2c->set_addr(L3GD20_ADDRESS);
    
    switch(range)
    {
    case L3DS20_RANGE_250DPS:
      tmp = i2c_smbus_write_byte_data(this->i2c->file, L3GD20_REGISTER_CTRL_REG4, 0x00);
      break;
    case L3DS20_RANGE_500DPS:
      tmp = i2c_smbus_write_byte_data(this->i2c->file, L3GD20_REGISTER_CTRL_REG4, 0x10);
      break;
    case L3DS20_RANGE_2000DPS:
      tmp = i2c_smbus_write_byte_data(this->i2c->file, L3GD20_REGISTER_CTRL_REG4, 0x20);
      break;
    }

    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        exit(1);
    }

    this->range = range;
}

void I2CL3GD20Gyro::read(vector3d* data)
{
    int tmp;
    __u8 buf[6];
    
    this->i2c->set_addr(L3GD20_ADDRESS);
    
    tmp = i2c_smbus_read_i2c_block_data(this->i2c->file, L3GD20_REGISTER_OUT_X_L|0x80, 6, (__u8*) buf);
    if (tmp != 6) {
        cout << "Error reading gyro data: " << tmp << endl;
        exit(1);
    }
    
    data->x = (int16_t)( buf[0] | (buf[1] << 8) );
    data->y = (int16_t)( buf[2] | (buf[3] << 8) );
    data->z = (int16_t)( buf[4] | (buf[5] << 8) );

    switch(this->range)
    {
    case L3DS20_RANGE_250DPS:
      data->x *= L3GD20_SENSITIVITY_250DPS;
      data->y *= L3GD20_SENSITIVITY_250DPS;
      data->z *= L3GD20_SENSITIVITY_250DPS;
      break;
    case L3DS20_RANGE_500DPS:
      data->x *= L3GD20_SENSITIVITY_500DPS;
      data->y *= L3GD20_SENSITIVITY_500DPS;
      data->z *= L3GD20_SENSITIVITY_500DPS;
      break;
    case L3DS20_RANGE_2000DPS:
      data->x *= L3GD20_SENSITIVITY_2000DPS;
      data->y *= L3GD20_SENSITIVITY_2000DPS;
      data->z *= L3GD20_SENSITIVITY_2000DPS;
      break;
    }
}
