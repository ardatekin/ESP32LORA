# ESP32 LoRa to Bluetooth Project

## Purpose of the Project

The purpose of this project is to enable remote data retrieval using LoRa communication and deliver the data to a Bluetooth-connected phone. The project supports various sensors, including temperature, humidity, and relay control. Commands are sent from a Bluetooth serial terminal on the phone to the LoRa sender, which communicates with the LoRa receiver to fetch the requested data.

---

## Prerequisites

To build and run this project, ensure you have the following tools and frameworks installed:

1. **VS Code**:
   - Ideally running under Linux for better compatibility.
2. **VS Code Extensions**:
   - [C/C++ Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
   - [ESP-IDF Extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
   - [Makefile Tools Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.makefile-tools)
   - [CMake Tools Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)

   These extensions can be downloaded and installed through VSCode's Extensions section.
3. **ESP-IDF Framework**:
   - Version 5.4 or above.
4. **ESP-IDF Arduino Libraries**:
   - Clone the Arduino libraries into the `components` directory:
     ```bash
     git clone https://github.com/espressif/arduino-esp32.git components/arduino
     ```

---

### Available Sensors, Modules, and Commands

This project is built using the following hardware components:

- **MCU**: ESP32 Development Board
- **LoRa Module**: RFM95, which uses Semtech's SX1276 chip
- **Temperature and Humidity Sensor**: SHT3x
- **LCD Screen**: Adafruit_ST7789 Compatible LCD Screen (such as Ideaspark ESP32 Board)

#### Commands

- **Temperature Sensor**: Fetch temperature data using the command `GET TEMP`.
- **Humidity Sensor**: Fetch humidity data using the command `GET HUMI`.
- **Relay Control**: Control the relay using commands like `GET RELAY_OPEN` and `GET RELAY_CLOSE`.

The data is fetched by sending these commands via a Bluetooth serial terminal connected to the LoRa sender. The LoRa sender communicates with the LoRa receiver to retrieve the requested data and sends the response back to the phone.

---

## Pin Wiring and Configuration

The pin mapping for the available sensors and modules is defined in the `Config.h` file. This file contains the GPIO pin assignments for the ESP32 development board, including connections for the LoRa module, temperature and humidity sensor (SHT3x), relay, and optional Adafruit_ST7789 compatible LCD screen.

Additionally, the mode of LoRa communication is configured in the `PROJECT_ROOT/CMakeLists.txt` file. By default, the project is set to `LORA_RECEIVER` mode. To switch to `LORA_SENDER` mode, comment out or remove the `LORA_RECEIVER` macro definition.

The enablement of components is also configured by enabling or disabling the following macro definitions in the `PROJECT_ROOT/CMakeLists.txt` file:
```cmake
# Global macro definitions
set(EXTRA_GLOBAL_DEFINES
    #   ADAFRUIT_LCD_ST7789_ENABLED
        LORA_RECEIVER
        RELAY_ENABLED
        BTCLASSIC_ENABLED
        SHT3X_ENABLED
)

To apply changes:
1. Modify the macro definitions in the `CMakeLists.txt` file as needed.
2. Perform a full clean, build, and flash process to update the ESP32 board:
   ```bash
   idf.py fullclean
   idf.py build
   idf.py flash
   ```

   Alternatively, you can use the ESP-IDF taskbar in the VSCode IDE to perform these actions:
   - **Clean**: Click the "Full Clean" button on the ESP-IDF taskbar.
   - **Build**: Click the "Build Project" button on the ESP-IDF taskbar.
   - **Flash**: Click the "Flash Device" button on the ESP-IDF taskbar.

Ensure that the wiring matches the pin definitions in `Config.h` and that the correct components are enabled in the `CMakeLists.txt` file to avoid any issues during operation.

---

## How to Use the Project

1. **Setup the Environment**:
   - Install the prerequisites listed above.
   - Clone this repository and open it in VS Code.

2. **Build the Project**:
   - Use the ESP-IDF extension in VS Code to configure and build the project.

3. **Flash the Firmware**:
   - Flash the firmware to your ESP32 device using the ESP-IDF tools.

4. **Connect via Bluetooth**:
   - Use a Bluetooth serial terminal app on your phone to connect to the ESP32 device.

5. **Send Commands**:
   - Send commands like `GET TEMP`, `GET HUMI`, or `GET RELAY_OPEN` to fetch data or control the relay.

6. **Receive Data**:
   - The requested data will be sent back to the Bluetooth terminal on your phone.

---

## Project Folder Structure

```
├── CMakeLists.txt
├── components
│   ├── Adafruit_GFX_Library
│   ├── Adafruit_SHT31
│   ├── Adafruit_BusIO
│   ├── Adafruit_ST7789_Library
│   ├── RFM95Lora
│   ├── Relay
│   ├── BTClassic
│   ├── SHT3X
│   ├── arduino
├── main
│   ├── CMakeLists.txt
│   ├── main.cpp
│   ├── CommandProcessor.cpp
│   └── CommandProcessor.h
├── docs
└── README.md
```

---

## License

This project is licensed under the LGPL-3.0-or-later license. See the individual source files for more details.
