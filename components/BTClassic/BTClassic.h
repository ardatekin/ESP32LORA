/*!
 *  @file BTClassic.h
 *
 *  @brief Header file for the Bluetooth Classic Module
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

#ifndef BTCLASSIC_H
#define BTCLASSIC_H

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "nvs_flash.h"
#include <string>
#include "CommandProcessor.h"


#define SPP_DATA_LEN 128

class BTClassic {
public:
    BTClassic(CommandProcessor* commandProcessor); // Constructor accepting CommandProcessor
    void Init(); // Initialize Bluetooth
    void Close(); // Close Bluetooth
    void SendResponse(const std::string& response);
    void HandleLoraReceiverMode(const std::string& response); // Handle LORA_RECEIVER mode
    void HandleLoraSenderMode(const std::string& command);    // Handle LORA_SENDER mode

private:
    static void EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
    static uint32_t connectionHandle;
    static BTClassic* instance; // Static pointer to BTClassic instance
    CommandProcessor* commandProcessor; // Pointer to CommandProcessor
};

#endif // BTCLASSIC_H
