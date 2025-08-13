#include <I2CLidar.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <time.h>
#include <thread>

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#ifdef __cplusplus
}
#endif

using namespace std;

I2CLidar::I2CLidar(I2C* i2c) : initializing_interval(100), acquiring_interval(18), ready_interval(18), retry_interval(5)
{
    this->i2c = i2c;

    this->value = -1;

    this->state = LIDAR_STATE_NONE;
    this->state_start = chrono::high_resolution_clock::now();

    this->internal_initialize();
}

void I2CLidar::internal_initialize()
{
    int tmp;
    
    i2c->set_addr(LIDAR_ADDRESS);
    
    tmp = i2c_smbus_read_byte_data(this->i2c->file, LIDAR_REG_STATUS);
    if (tmp < 0) {
        cout << "Error reading status: " << tmp << endl;
        exit(1);
    }
    
    /*
    if (tmp != 0) {
        if (tmp & LIDAR_STAT_BUSY) cout << "LIDAR WARN: Busy" << endl;
        if (tmp & LIDAR_STAT_REF_OVERFLOW) cout << "LIDAR WARN: Reference overflow" << endl;
        if (tmp & LIDAR_STAT_SIG_OVERFLOW) cout << "LIDAR WARN: Signal overflow" << endl;
        if (tmp & LIDAR_STAT_PIN_STATUS) cout << "LIDAR WARN: Pin status" << endl;
        if (tmp & LIDAR_STAT_SECOND_PEAK) cout << "LIDAR WARN: Second peak" << endl;
        if (tmp & LIDAR_STAT_TIMESTAMP) cout << "LIDAR WARN: Timestamp" << endl;
        if (tmp & LIDAR_STAT_INVALID) cout << "LIDAR WARN: SignaliInvalid" << endl;
        if (tmp & LIDAR_STAT_EYE_SAFE) cout << "LIDAR WARN: Eye safetey" << endl;
    }
    */

    this->state = LIDAR_STATE_INITIALIZING;
    this->state_start = chrono::high_resolution_clock::now();
}

int I2CLidar::read()
{
    chrono::high_resolution_clock::duration state_duration = 
        chrono::high_resolution_clock::now() - this->state_start;
    
    if ( this->state == LIDAR_STATE_NONE ) {
        if (state_duration > initializing_interval) {
            this->internal_initialize();
        }
    } else if ( this->state == LIDAR_STATE_INITIALIZING) {
        if (state_duration > initializing_interval) {
            this->internal_acquire();
        }
    } else if ( this->state == LIDAR_STATE_ACQUIRING) {
        if (state_duration > acquiring_interval) {
            this->internal_read();
        }
    } else if (this->state == LIDAR_STATE_READY) {
        if (state_duration > ready_interval) {
            this->internal_acquire();
        }
    }

    return this->value;
}

void I2CLidar::internal_acquire()
{
    int tmp;

    this->i2c->set_addr(LIDAR_ADDRESS);

    tmp = i2c_smbus_write_byte_data(this->i2c->file, LIDAR_REG_COMMAND, LIDAR_CMD_ACQUIRE);
    if (tmp < 0) {
        cout << "Error writing mode: " << tmp << endl;
        return;
    }
    
    this->state = LIDAR_STATE_ACQUIRING;
    this->state_start = chrono::high_resolution_clock::now();
}

void I2CLidar::internal_read()
{
    int tmp;
    
    this->i2c->set_addr(LIDAR_ADDRESS);

    tmp = i2c_smbus_read_byte_data(this->i2c->file, LIDAR_REG_DATA_H);
    for ( int i = 0; i < 10 && tmp < 0; i++ ) {
        this_thread::sleep_for(retry_interval);
        tmp = i2c_smbus_read_byte_data(this->i2c->file, LIDAR_REG_DATA_H);
    }
    if (tmp < 0) {
        cout << "Error reading data high: " << tmp << endl;
        return;
    }
    int bh = tmp;

    
    tmp = i2c_smbus_read_byte_data(this->i2c->file, LIDAR_REG_DATA_L);
    for ( int i = 0; i < 10 && tmp < 0; i++ ) {
        this_thread::sleep_for(retry_interval);
        tmp = i2c_smbus_read_byte_data(this->i2c->file, LIDAR_REG_DATA_L);
    }
    if (tmp < 0) {
        cout << "Error reading data low: " << tmp << endl;
        return;
    }
    int bl = tmp;

    int v = (int16_t)( bl | ((int16_t)bh << 8) );

    if (v == 0) {
        //cout << "LIDAR WARN: Read 0" << endl;
    } else {
        this->value = v;
        //cout << "LIDAR VALUE: " << this->value << endl;
    }
    this->state = LIDAR_STATE_READY;
    this->state_start = chrono::high_resolution_clock::now();
}

