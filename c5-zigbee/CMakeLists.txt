cmake_minimum_required(VERSION 3.16)

# Support local ESP-Zigbee-SDK path for development
if(DEFINED ENV{ESP_ZIGBEE_SDK_PATH})
    set(EXTRA_COMPONENT_DIRS $ENV{ESP_ZIGBEE_SDK_PATH}/components)
    message(STATUS "Using local ESP-Zigbee-SDK: $ENV{ESP_ZIGBEE_SDK_PATH}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(c5_zigbee_coordinator VERSION 1.0.0)

# Support machine-specific overrides
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.local")
    message(STATUS "Loading local SDK config overrides")
endif()
