/*!
 *  @file RFM95Lora.h
 *
 *  @brief Header file for the RFM95 LoRa Module
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

#include <string.h>
#include <sys/reent.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"
#include "esp_log.h"

class LoRa {
public:
    LoRa() : spi(nullptr) {}
    void setFrequency(double freq_hz);                                                                                                                                                
    esp_err_t init();
    esp_err_t send(const uint8_t* data, int length);
    int receive(uint8_t* buffer, int bufferSize);
    uint8_t debugReadRegister(uint8_t reg);

private:
    spi_device_handle_t spi;
    void reset();
public:
    esp_err_t writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
};