cmake_minimum_required(VERSION 3.5)

idf_component_register(
    SRCS 
        "main.cpp"
        "CommandProcessor.cpp"
    INCLUDE_DIRS 
        "."
        "${PROJECT_ROOT}/components/Adafruit_GFX_Library"
        "${PROJECT_ROOT}/components/Adafruit_SHT31"
        "${PROJECT_ROOT}/components/RFM95Lora"
        "${PROJECT_ROOT}/components/Relay"
        "${PROJECT_ROOT}/components/BTClassic"
        "${PROJECT_ROOT}/components/SHT3X"
        "${PROJECT_ROOT}/components/Voltage"
    REQUIRES 
        bt 
        nvs_flash 
        driver 
        Adafruit_BusIO 
        arduino
)

