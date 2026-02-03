cmake_minimum_required(VERSION 3.16)

# Add shared Zigbee coordinator component
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/../components"
)

# Support local ESP-Zigbee-SDK path for development
if(DEFINED ENV{ESP_ZIGBEE_SDK_PATH})
    list(APPEND EXTRA_COMPONENT_DIRS $ENV{ESP_ZIGBEE_SDK_PATH}/components)
    message(STATUS "Using local ESP-Zigbee-SDK: $ENV{ESP_ZIGBEE_SDK_PATH}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(h2_zigbee_coordinator VERSION 1.0.0)

# Support machine-specific overrides
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.local")
    message(STATUS "Loading local SDK config overrides")
endif()
