/*!
 *  @file Relay.cpp
 *
 *  @brief Source file for the Relay Module
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

#include "Relay.h"
#include <stdio.h> // For printf debugging
#include "Config.h"

Relay::Relay() 
{

}

void Relay::Init()
{
    printf("Configuring GPIO for Relay...\n");
    esp_rom_gpio_pad_select_gpio((gpio_num_t)RELAY_PIN);
    gpio_set_direction((gpio_num_t)RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)RELAY_PIN, 1); // Ensure relay is off initially
    printf("GPIO %d configured for Relay.\n", RELAY_PIN);
}

void Relay::Open()
{
    printf("Turning Relay ON (GPIO %d).\n", RELAY_PIN);
    gpio_set_level((gpio_num_t)RELAY_PIN, 0); // Turn relay on
}

void Relay::Close()
{
    printf("Turning Relay OFF (GPIO %d).\n", RELAY_PIN);
    gpio_set_level((gpio_num_t)RELAY_PIN, 1); // Turn relay off
}
