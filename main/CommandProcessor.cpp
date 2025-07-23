/*!
 *  @file CommandProcessor.cpp
 *
 *  @brief Source file for the Command Processor
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

#include "CommandProcessor.h"
#include <sstream>
#include <stdio.h> // For printf debugging

#ifdef SHT3X_ENABLED
#include "SHT3X.h" // Include SHT3X only in the implementation file
#endif

#ifdef RELAY_ENABLED
#include "Relay.h" // Include Relay only in the implementation file
#endif

#ifdef BTCLASSIC_ENABLED
#include "BTClassic.h"
#endif

CommandProcessor::CommandProcessor()
#ifdef BTCLASSIC_ENABLED
    #ifdef LORA_RECEIVER
    : btClassic(new BTClassic(this)) // Pass the CommandProcessor instance to BTClassic
    #endif
#endif
{
#ifdef RELAY_ENABLED
    relay = nullptr;
#endif
#ifdef SHT3X_ENABLED
    sht3x = nullptr;
#endif
}

CommandProcessor::~CommandProcessor()
{
#ifdef BTCLASSIC_ENABLED
    #ifdef LORA_RECEIVER
    delete btClassic; // Clean up the BTClassic instance
    #endif
#endif
#ifdef RELAY_ENABLED
    relay = nullptr; // Relay is not owned by CommandProcessor, so no deletion
#endif
#ifdef SHT3X_ENABLED
    sht3x = nullptr; // SHT3X is not owned by CommandProcessor, so no deletion
#endif
}

void CommandProcessor::Init()
{
    // Initialization logic if needed
}

void CommandProcessor::SetSHT3X(SHT3X* sht3x)
{
#ifdef SHT3X_ENABLED
    this->sht3x = sht3x; // Inject the SHT3X dependency
#endif
}

void CommandProcessor::SetRelay(Relay* relay)
{
#ifdef RELAY_ENABLED
    this->relay = relay; // Inject the Relay dependency
#endif
}

CommandProcessor::SensorType CommandProcessor::ParseCommand(const std::string& command)
{
    std::istringstream stream(command);
    std::string action, sensorType;
    stream >> action >> sensorType;

    if (action == "GET") {
        if (sensorType == "TEMP") return TEMP;
        if (sensorType == "HUMI") return HUMI;
    } else if (action == "SET") {
        if (sensorType == "RELAY_OPEN") return RELAY_OPEN;
        if (sensorType == "RELAY_CLOSE") return RELAY_CLOSE;
        if (sensorType == "BT_OPEN") return BT_OPEN;
        if (sensorType == "BT_CLOSE") return BT_CLOSE;
        if (sensorType == "RESET") return RESET; // Add RESET command
    }
    return UNKNOWN;
}

std::string CommandProcessor::ProcessCommand(const std::string& command)
{
    printf("Processing command: %s\n", command.c_str());
    SensorType sensorType = ParseCommand(command);
    switch (sensorType) {
        case TEMP: {
#ifdef SHT3X_ENABLED
            if (sht3x) {
                float temperature = sht3x->GetTemperature();
                if (!isnan(temperature)) {
                    char tempBuffer[32];
                    snprintf(tempBuffer, sizeof(tempBuffer), "Temperature: %.2f°C\n", temperature);
                    return std::string(tempBuffer); // Valid response
                } else {
                    return "Error: Failed to read temperature"; // Error response
                }
            }
#endif
            return "Error: Temperature sensor not enabled"; // Error response
        }
        case HUMI: {
#ifdef SHT3X_ENABLED
            if (sht3x) {
                float humidity = sht3x->GetHumidity();
                if (!isnan(humidity)) {
                    char humiBuffer[32];
                    snprintf(humiBuffer, sizeof(humiBuffer), "Humidity: %.2f%%\n", humidity);
                    return std::string(humiBuffer); // Valid response
                } else {
                    return "Error: Failed to read humidity\n"; // Error response
                }
            }
#endif
            return "Error: Humidity sensor not enabled\n"; // Error response
        }
        case RELAY_OPEN:
#ifdef RELAY_ENABLED
            if (relay) {
                relay->Open();
                return "Relay Opened\n"; // Valid response
            } else {
                printf("Relay not initialized.\n");
            }
#endif
            return "Error: Relay not initialized\n"; // Error response
        case RELAY_CLOSE:
#ifdef RELAY_ENABLED
            if (relay) {
                relay->Close();
                return "Relay Closed"; // Valid response
            } else {
                printf("Relay not initialized.\n");
            }
#endif
            return "Error: Relay not initialized\n"; // Error response
        case BT_OPEN:
#ifdef BTCLASSIC_ENABLED
#ifdef LORA_RECEIVER
            btClassic->Init(); // Initialize Bluetooth for LORA_RECEIVER
            return "Bluetooth initialized\n"; 
#endif
            
#endif
            return "Error: Bluetooth not initialized\n"; // Error response
        case BT_CLOSE:
#ifdef BTCLASSIC_ENABLED
#ifdef LORA_RECEIVER
        if (btClassic) {
            btClassic->Close(); // Close Bluetooth
            return "Bluetooth Closed\n"; // Valid response
        } else {
            return "Error: Bluetooth not initialized\n"; // Error response
        }
#endif
#endif
            return "Error: Bluetooth not initialized\n"; // Error response
        case RESET:
            printf("Rebooting ESP32...\n");
            esp_restart(); // Reboot the ESP32
            return "ESP32 Rebooting...\n"; // Valid response
        default:
            return "Error: Invalid Command"; // Error response
    }
}

void CommandProcessor::EnqueueCommand(const std::string& command)
{
    std::lock_guard<std::mutex> lock(queueMutex);
    commandQueue.push_back(command);
}

std::string CommandProcessor::DequeueCommand()
{
    std::lock_guard<std::mutex> lock(queueMutex);
    if (!commandQueue.empty()) {
        std::string command = commandQueue.front();
        commandQueue.erase(commandQueue.begin());
        return command;
    }
    return ""; // Return an empty string if the queue is empty
}
