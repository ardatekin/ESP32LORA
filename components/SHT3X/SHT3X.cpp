/*!
 *  @file SHT3X.cpp
 *
 *  @brief Source file for the SHT3X Digital Humidity & Temperature Sensor
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

#include "SHT3X.h"
#include <stdio.h> // For debugging

SHT3X::SHT3X(uint8_t sdaPin, uint8_t sclPin, uint8_t i2cAddr)
    : customWire(0) // Initialize customWire first
{
    printf("Initializing TwoWire with SDA=%d, SCL=%d\n", sdaPin, sclPin);
    if (!customWire.begin(sdaPin, sclPin)) { // Initialize I2C with custom pins
        printf("Failed to initialize TwoWire with SDA=%d, SCL=%d\n", sdaPin, sclPin);
    } else {
        printf("TwoWire initialized successfully with SDA=%d, SCL=%d\n", sdaPin, sclPin);
    }

    // Initialize Adafruit_SHT31 after customWire is fully initialized
    sht31 = Adafruit_SHT31(&customWire);
}

bool SHT3X::Init() 
{
    printf("Initializing SHT31 sensor...\n");
    if (!sht31.begin()) { // Initialize the sensor
        printf("Failed to initialize SHT31 sensor.\n");
        return false;
    }
    printf("SHT31 sensor initialized successfully.\n");
    return true;
}

float SHT3X::GetTemperature() 
{
    printf("Entering SHT3X::GetTemperature\n");
    float temperature = sht31.readTemperature(); // Read temperature
    if (isnan(temperature)) { // Check for invalid data
        printf("Failed to read temperature from SHT31 sensor.\n");
        return NAN; // Return NaN to indicate failure
    }
    printf("Temperature read successfully: %.2f°C\n", temperature);
    return temperature;
}

float SHT3X::GetHumidity() 
{
    printf("Entering SHT3X::GetHumidity\n");
    float humidity = sht31.readHumidity(); // Read humidity
    if (isnan(humidity)) { // Check for invalid data
        printf("Failed to read humidity from SHT31 sensor.\n");
        return NAN; // Return NaN to indicate failure
    }
    printf("Humidity read successfully: %.2f%%\n", humidity);
    return humidity;
}
