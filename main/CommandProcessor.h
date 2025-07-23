/*!
 *  @file CommandProcessor.h
 *
 *  @brief Header file for the Command Processor
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

#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <string>
#include <vector>
#include <mutex>

#ifdef SHT3X_ENABLED
class SHT3X; // Forward declaration of SHT3X class
#endif

#ifdef RELAY_ENABLED
class Relay; // Forward declaration of Relay class
#endif

#ifdef BTCLASSIC_ENABLED
#ifdef LORA_RECEIVER
class BTClassic; // Forward declaration of BTClassic class
#endif
#endif

class CommandProcessor {
public:
    enum SensorType {
        TEMP,
        HUMI,
        RELAY_OPEN,
        RELAY_CLOSE,
        BT_OPEN,
        BT_CLOSE,
        RESET,
        UNKNOWN
    };

    CommandProcessor();
    ~CommandProcessor();
    void Init();
    void SetSHT3X(SHT3X* sht3x); // Dependency injection for SHT3X
    void SetRelay(Relay* relay); // Dependency injection for Relay
    SensorType ParseCommand(const std::string& command);
    std::string ProcessCommand(const std::string& command);

    void EnqueueCommand(const std::string& command); // Add command to the queue
    std::string DequeueCommand(); // Retrieve and remove the next command from the queue

private:
#ifdef RELAY_ENABLED
    Relay* relay; // Pointer to Relay object
#endif
#ifdef SHT3X_ENABLED
    SHT3X* sht3x; // Pointer to SHT3X object
#endif
#ifdef BTCLASSIC_ENABLED
    #ifdef LORA_RECEIVER
    BTClassic* btClassic;
    #endif
#endif

    std::vector<std::string> commandQueue; // FIFO queue for commands
    std::mutex queueMutex; // Mutex to protect access to the queue
};

#endif // COMMAND_PROCESSOR_H
