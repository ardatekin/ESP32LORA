/*!
 *  @file main.cpp
 *
 *  @brief Main application file for the ESP32 LoRa Project
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

#include <stdio.h>
#include <string.h>
#include <vector>
#include <mutex>

#include "Config.h"

#ifdef ADAFRUIT_LCD_ST7789_ENABLED
    #include "Adafruit_GFX.h"
    #include "Adafruit_ST7789.h"
#endif

#include "RFM95Lora.h"
#ifndef LORA_RECEIVER
    #define LORA_SENDER
#endif

#include "CommandProcessor.h"
#include "BTClassic.h"
#include "SHT3X.h"
#include "Relay.h"


#ifdef ADAFRUIT_LCD_ST7789_ENABLED
void clearScreen(Adafruit_ST7789& lcd) 
{
    lcd.fillScreen(ST77XX_BLACK);
}

void clearArea(Adafruit_ST7789& lcd, int x, int y, int w, int h) 
{
    lcd.fillRect(x, y, w, h, ST77XX_BLACK);
    lcd.setCursor(x, y);    
}
#endif


extern "C" void app_main(void)
{
    #ifdef LORA_SENDER
    printf("Starting Lora Sender application\n");
    #else
    printf("Starting Lora Receiver application\n");
    #endif

    CommandProcessor commandProcessor;
    commandProcessor.Init(); // Initialize command processor

#ifdef SHT3X_ENABLED
    SHT3X sht3x(SHT3X_SDA_PIN, SHT3X_SCL_PIN);
    if (sht3x.Init()) {
        commandProcessor.SetSHT3X(&sht3x); // Inject SHT3X into CommandProcessor
    } else {
        printf("Failed to initialize SHT3X sensor.\n");
    }
#endif

#ifdef RELAY_ENABLED
    Relay relay;
    relay.Init();
    commandProcessor.SetRelay(&relay); // Inject Relay into CommandProcessor
#endif

    // Initialize Bluetooth and pass commandProcessor
    BTClassic btClassic(&commandProcessor);
    btClassic.Init();

    // Initialize Lora
    LoRa lora;
    if (lora.init() != ESP_OK) {
        printf("LoRa initialization failed\n");
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds
        return;
    } else {
        printf("LoRa initialization successful\n");
        lora.debugReadRegister(0x42);  // Should print 0x12 if SPI is working
    }

#ifdef  ADAFRUIT_LCD_ST7789_ENABLED
    // Create an instance of the display
    Adafruit_ST7789 lcd = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);
    // Initialize the display
    lcd.init(170, 320); // For ST7789
    // Set rotation and fill screen
    lcd.setRotation(1);
    lcd.fillScreen(ST77XX_BLACK);
    // Display some text
    lcd.setTextColor(ST77XX_WHITE);
    lcd.setTextSize(2);
    lcd.setCursor(0, 0);
    lcd.println("Hello, World!");
    lcd.setCursor(0, 30);
    lcd.print("Received: ");
#endif

    while (1) {
        #ifdef LORA_SENDER
            // Get the next command from the CommandProcessor's queue
            std::string command = commandProcessor.DequeueCommand();
            if (!command.empty()) {
                printf("Command dequed: %s\n", command.c_str());
                lora.send(reinterpret_cast<const uint8_t*>(command.c_str()), command.length());
                printf("Command sent: %s\n", command.c_str());

                // Wait for a response from the LoRa receiver
                uint8_t rxBuffer[64] = {0};
                int len = lora.receive(rxBuffer, sizeof(rxBuffer) - 1);
                if (len > 0) {
                    rxBuffer[len] = '\0';  // Null-terminate the received string
                    std::string loraResponse((const char*)rxBuffer);
                    printf("Received Response from LoRa Receiver: %s\n", loraResponse.c_str());

                    // Send the response back to the Android phone via Bluetooth
                    btClassic.SendResponse(loraResponse);
                } else {
                    printf("No response received from LoRa Receiver.\n");
                }
            } else {
                printf("No commands in the queue.\n");
            }
            vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds before sending the next command
        #else
            uint8_t rxBuffer[64] = {0};
            int len = lora.receive(rxBuffer, sizeof(rxBuffer) - 1);
            if (len > 0) {
                rxBuffer[len] = '\0';  // Null-terminate the received string
                std::string receivedCommand((const char*)rxBuffer);
                std::string response = commandProcessor.ProcessCommand(receivedCommand);

                printf("Received Command: %s\n", receivedCommand.c_str());
                printf("Response: %s\n", response.c_str());

                // Only send a response back to the LoRa sender if it is valid
                if (response.find("Error:") != 0) { // Check if the response does not start with "Error:"
                    lora.send(reinterpret_cast<const uint8_t*>(response.c_str()), response.length());
                    printf("Sent Response to LoRa Sender: %s\n", response.c_str());
                } else {
                    printf("No valid response to send back to LoRa Sender.\n");
                }

                #ifdef ADAFRUIT_LCD_ST7789_ENABLED
                // Clear the previous message by filling the area with the background color
                clearArea(lcd, 0, 60, 320, 30);
                lcd.print("Response: ");
                lcd.print(response.c_str());
                #endif
            }
        #endif
    }

    // Infinite loop to keep the application running
    while (1) {
        // Add any periodic tasks or leave it empty
        vTaskDelay(pdMS_TO_TICKS(1000)); // Sleep for 1 second
    }


}