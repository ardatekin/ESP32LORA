cmake_minimum_required(VERSION 3.5)

set(SDK_ROOT $ENV{IDF_PATH}) # Dynamically retrieve ESP-IDF path from environment

idf_component_register(SRCS "RFM95Lora.cpp"
                    INCLUDE_DIRS "."
                    "${PROJECT_ROOT}/main" 
                    "${SDK_ROOT}/components/esp_driver_spi/include"
                    "${SDK_ROOT}/components/esp_driver_gpio/include")

project(RFM95Lora_Library)

