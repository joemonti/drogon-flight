/*
 * I2CServo.h
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
#ifndef __I2CSERVO_H__
#define __I2CSERVO_H__

#include "I2C.h"

#define NUM_SERVO_CHANNELS 4

class I2CServo {
  public:
    I2CServo(I2C* i2c);
    
    void setFreq(float freq);
    void setMicros(int channel, int micros);

  private:
    I2C* i2c;
    float offPerMicro;

    int microsValues[NUM_SERVO_CHANNELS];
};

#endif // __I2CSERVO_H__
