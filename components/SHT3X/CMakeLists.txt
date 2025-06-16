cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "SHT3X.cpp"
                    INCLUDE_DIRS 
                    "."
                    "${PROJECT_ROOT}/main" 
                    REQUIRES Adafruit_SHT31)

project(SHT3X)
