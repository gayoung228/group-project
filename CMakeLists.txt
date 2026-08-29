cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME car_project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 기본 펌웨어와 단독 테스트 중 하나만 선택한다.
# 사용 예: cmake --preset Debug -DROVER_FIRMWARE_MODE=HEADING_TEST
set(ROVER_FIRMWARE_MODE "ROVER" CACHE STRING
    "Firmware mode: ROVER, HEADING_TEST, IR_TEST, VL53_TEST")
set_property(CACHE ROVER_FIRMWARE_MODE PROPERTY STRINGS
    ROVER HEADING_TEST IR_TEST VL53_TEST)

set(ROVER_COMMON_SOURCES
    app/ap_main.c
    hw/control/heading_control.c
    hw/control/mission_control.c
    hw/control/obstacle_avoidance.c
    hw/control/speed_bump_control.c
    hw/driver/debug_uart.c
    hw/driver/drive.c
    hw/driver/encoder.c
    hw/driver/ir_remote.c
    hw/driver/motor.c
    hw/driver/mpu6050.c
    hw/driver/vl53l0x.c
    hw/driver/wheel.c
)

if(ROVER_FIRMWARE_MODE STREQUAL "ROVER")
    set(ROVER_APP_SOURCE
        app/rover_app.c
        app/wheel_calibration.c)
    set(ROVER_APP_DEFINITIONS APP_ROVER)
elseif(ROVER_FIRMWARE_MODE STREQUAL "HEADING_TEST")
    set(ROVER_APP_SOURCE
        app/tests/heading_drive_test.c
        app/tests/vl53l0x_test.c)
    set(ROVER_APP_DEFINITIONS TEST_HEADING_DRIVE TEST_VL53L0X)
elseif(ROVER_FIRMWARE_MODE STREQUAL "IR_TEST")
    set(ROVER_APP_SOURCE app/tests/ir_remote_test.c)
    set(ROVER_APP_DEFINITIONS TEST_IR_REMOTE)
elseif(ROVER_FIRMWARE_MODE STREQUAL "VL53_TEST")
    set(ROVER_APP_SOURCE app/tests/vl53l0x_test.c)
    set(ROVER_APP_DEFINITIONS TEST_VL53L0X)
else()
    message(FATAL_ERROR "Unknown ROVER_FIRMWARE_MODE=${ROVER_FIRMWARE_MODE}")
endif()

message(STATUS "Rover firmware mode: ${ROVER_FIRMWARE_MODE}")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${ROVER_COMMON_SOURCES}
    ${ROVER_APP_SOURCE}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    app
    app/tests
    hw
    hw/driver
    hw/control
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${ROVER_APP_DEFINITIONS}
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)
