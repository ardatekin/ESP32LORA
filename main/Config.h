/*!
 *  @file Config.h
 *
 *  @brief Configuration file for the ESP32LORA project
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

#pragma once

#ifdef LORA_RECEIVER
    #pragma message("LORA_RECEIVER is defined")
    #define SPP_SERVER_NAME "ESP32_BT_RECEIVER"
#else
    #pragma message("LORA_RECEIVER is NOT defined")    
    #define SPP_SERVER_NAME "ESP32_BT_SENDER"
#endif

// RFM95 (SEMTEC SX1276) LORA PIN Definitions
#define LORA_PIN_NUM_MISO 19
#define LORA_PIN_NUM_MOSI 27
#define LORA_PIN_NUM_CLK  5
#define LORA_PIN_NUM_CS   22
#define LORA_PIN_NUM_RST  21
#define LORA_PIN_NUM_DIO0 26

#define LORA_FREQ 0xE4C000      // 868 MHz for EU
//#define LORA_FREQ 0xE6CC00    // 915 MHz for North America
//#define LORA_FREQ 0xD90000    // 433 MHz for Asia

#ifdef ADAFRUIT_LCD_ST7789_ENABLED
    #pragma message("ADAFRUIT_LCD_ST7789_ENABLED is defined")
    // Define the pins for the display
    // Note: ESP-PROG JTAG uses GPIO15 for TDO connection 
    // which conflicts with CS connection of Adafruit display of Ideaspark board.
    // So, when ADAFRUIT_LCD_ST7789 is enabled, JTAG cannnot be used
    #define LCD_CS     15
    #define LCD_RST    4
    #define LCD_DC     2
#endif

#ifdef SHT3X_ENABLED
    #pragma message("SHT3X_ENABLED is defined")
    #define SHT3X_SDA_PIN 4 // Custom SDA pin
    #define SHT3X_SCL_PIN 15 // Custom SCL pin
#endif

#ifdef RELAY_ENABLED
    #pragma message("RELAY_ENABLED is defined")
    #define RELAY_PIN 25 // Define the relay pin number here
#endif

#define VOLTAGE_READER_PIN ADC1_CHANNEL_0 // ADC pin for voltage measurement