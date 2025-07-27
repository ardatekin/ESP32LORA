/*!
 *  @file Voltage.cpp
 *
 *  @brief Source file for the Voltage Module
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

#include "Voltage.h"
#include "Config.h"

Voltage::Voltage() 
{
    // Configure ADC1 channel 0 (GPIO36)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(VOLTAGE_READER_PIN, ADC_ATTEN_DB_11); // 0-3.6V range
}

Voltage::~Voltage() 
{
    
}

float Voltage::Read() 
{
    int raw = adc1_get_raw(VOLTAGE_READER_PIN); // Read raw ADC value
    float voltage = (raw / 4095.0) * 3.6 * 2;   // Convert to voltage (0-3.6V)
    return voltage;
}
