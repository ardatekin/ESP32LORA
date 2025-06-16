/*!
 *  @file SHT3X.h
 *
 *  @brief Header file for the SHT3X Digital Humidity & Temperature Sensor
 *
 *  @author Arda Tekin (https://www.linkedin.com/in/ardatekin)
 *
 *  This file is part of ESP32LORA project.
 *  
 *  ESP32LORA is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  ESP32LORA is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with ESP32LORA. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SHT3X_H
#define SHT3X_H

#include <Adafruit_SHT31.h>
#include <Wire.h> // Include Wire library for I2C
#include "Config.h"

class SHT3X {
public:
    SHT3X(uint8_t sdaPin, uint8_t sclPin, uint8_t i2cAddr = 0x44);
    bool Init();
    float GetTemperature();
    float GetHumidity();

private:
    TwoWire customWire; // Custom TwoWire instance
    Adafruit_SHT31 sht31; // Adafruit SHT31 instance
};

#endif // SHT3X_H
