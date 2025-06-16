# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Define the project root directory
set(PROJECT_ROOT "${CMAKE_SOURCE_DIR}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Global macro definitions
set(EXTRA_GLOBAL_DEFINES
#   ADAFRUIT_LCD_ST7789_ENABLED
    LORA_RECEIVER
    RELAY_ENABLED
    BTCLASSIC_ENABLED
    SHT3X_ENABLED
)

# Export macros to all components
foreach(DEFINE ${EXTRA_GLOBAL_DEFINES})
    add_compile_options("-D${DEFINE}")
endforeach()

project(esplora)

