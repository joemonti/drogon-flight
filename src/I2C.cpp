/*
 * I2C.cpp
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
#include <I2C.h>

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

#define DEV_NUM 1

using namespace std;

I2C::I2C(void) 
{
    char filename[20];  
    snprintf(filename, 19, "/dev/i2c-%d", DEV_NUM);
    this->file = open(filename, O_RDWR);
    if (this->file < 0) {
        cout << "Error opening device " << filename << ": " << this->file << endl;
        exit(1);
    }
    
    this->last_addr = -1;
}

void I2C::set_addr(int addr) 
{
    if (addr != this->last_addr) {
        int tmp;
        
        tmp = ioctl(this->file, I2C_SLAVE, addr);
        if (tmp < 0) {
            cout << "Error setting slave addr: " << tmp << endl;
            exit(1);
        }

        this->last_addr = addr;
    }
}

void I2C::close(void)
{
    ::close(this->file);
}
