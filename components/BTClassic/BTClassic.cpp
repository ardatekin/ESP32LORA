/*!
 *  @file BTClassic.cpp
 *
 *  @brief Source file for the Bluetooth Classic Module
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

#include "BTClassic.h"
#include <stdio.h>
#include <string.h>
#include "Config.h"

uint32_t BTClassic::connectionHandle = 0;
BTClassic* BTClassic::instance = nullptr; // Initialize static pointer

BTClassic::BTClassic(CommandProcessor* cProcessor)
{
    commandProcessor = cProcessor; 
    instance = this; // Set the static instance pointer
}

void BTClassic::Init()
{
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE); // Release BLE memory if not used
    if (ret != ESP_OK) {
        printf("Failed to release BLE memory: %s\n", esp_err_to_name(ret));
    }

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT(); // Ensure proper initialization
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        printf("Bluetooth controller initialization failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT); // Enable Classic Bluetooth mode
    if (ret != ESP_OK) {
        printf("Bluetooth controller enabling failed: %s\n", esp_err_to_name(ret));
        printf("Controller status: %d\n", esp_bt_controller_get_status());
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        printf("Bluedroid initialization failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        printf("Bluedroid enabling failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // Set Bluetooth scan mode to connectable and discoverable
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    esp_spp_register_callback(EventHandler);

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB, // Set the SPP mode to callback
        .enable_l2cap_ertm = true, // Enable L2CAP ERTM
        .tx_buffer_size = 0 // Default TX buffer size
    };
    esp_spp_enhanced_init(&spp_cfg); // Pass the configuration structure

    printf("Bluetooth initialized successfully\n");
}

void BTClassic::SendResponse(const std::string& response)
{
    if (connectionHandle) {
        esp_spp_write(connectionHandle, response.length(), (uint8_t*)response.c_str());
    }
}

// Return the response to the Bluetooth client
// This function is called when the device is in LORA_RECEIVER mode
void BTClassic::HandleLoraReceiverMode(const std::string& response)
{
    if (connectionHandle) {
        esp_spp_write(connectionHandle, response.length(), (uint8_t*)response.c_str());
        printf("Sent response to Bluetooth client: %s\n", response.c_str());
    }
}

void BTClassic::HandleLoraSenderMode(const std::string& command)
{
    if (commandProcessor) {
        commandProcessor->EnqueueCommand(command); // Use CommandProcessor's queue
        printf("Enqueued command for LORA_SENDER: %s\n", command.c_str());
    } else {
        printf("CommandProcessor is not initialized. Cannot enqueue command.\n");
    }
}

void BTClassic::EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    static char rx_buffer[128] = {0};
    std::string response;
    switch (event) {
        case ESP_SPP_SRV_OPEN_EVT:
            printf("Client connected. Handle: %ld\n", param->srv_open.handle);
            connectionHandle = param->srv_open.handle;
            break;
        case ESP_SPP_INIT_EVT:
            printf("SPP Initialized, starting server...\n");
            printf("Bluetooth device name %s\n", SPP_SERVER_NAME);
            esp_bt_gap_set_device_name(SPP_SERVER_NAME); // Set the device name for Bluetooth
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;
        case ESP_SPP_DATA_IND_EVT:
            memcpy(rx_buffer, param->data_ind.data, param->data_ind.len);
            rx_buffer[param->data_ind.len] = '\0'; // Null-terminate the received string
            printf("Received via Bluetooth: %s\n", rx_buffer);
            if (instance && instance->commandProcessor) { // Access commandProcessor via instance
                response = instance->commandProcessor->ProcessCommand(std::string(rx_buffer));
                printf("Response: %s\n", response.c_str());
                #ifdef LORA_RECEIVER
                instance->HandleLoraReceiverMode(response); // Handle LORA_RECEIVER mode
                #else
                instance->HandleLoraSenderMode(std::string(rx_buffer)); // Handle LORA_SENDER mode
                #endif
            }
            break;
        default:
            printf("Event not found \n");
            break;
    }
}

void BTClassic::Close() 
{
    printf("Closing Bluetooth...\n");

    // Uninitialize SPP
    esp_err_t ret = esp_spp_deinit();
    if (ret != ESP_OK) {
        printf("Failed to deinitialize SPP: %s\n", esp_err_to_name(ret));
    } else {
        printf("SPP deinitialized successfully.\n");
    }

    // Disable and deinitialize Bluedroid stack
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
        esp_bluedroid_disable();
        printf("Bluedroid disabled.\n");
    }
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        esp_bluedroid_deinit();
        printf("Bluedroid deinitialized.\n");
    }

    // Disable and deinitialize Bluetooth controller
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
        printf("Bluetooth controller disabled.\n");
    }
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_bt_controller_deinit();
        printf("Bluetooth controller deinitialized.\n");
    }

    // Reset the connection handle
    connectionHandle = 0;

    printf("Bluetooth closed successfully.\n");
}
