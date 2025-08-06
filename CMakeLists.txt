cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
# set your OS environment variable PICO_SDK_PATH to .pico-sdk
# EXAMPLE: export PICO_SDK_PATH=$HOME/.pico-sdk
# In Windows, search for "environment" in the start menu and add a new environment variable

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD SAMWISE CACHE STRING "Board type")
# Comment out the next line if you are building for a Pico breadboard
set (PICO_PLATFORM rp2350 CACHE STRING "Platform Type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

set (PROJECT blink)
project(blink C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Get the TinyUSB config file for the RP2350 and put it into include
#  git clone https://github.com/hathach/tinyusb.git
add_executable(${PROJECT}  ${PROJECT}.c src/rfm96.c src/ws2812.c src/mppt.c src/power.c 
    src/ads7830.c src/gps.c src/uart.c)

#  Comment this out if you're building for the PiCubed pinout breadboard
#add_compile_definitions(PICO)

target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT} pico_stdlib hardware_i2c hardware_spi hardware_uart 
    hardware_gpio
#    tinyusb_device tinyusb_board
    hardware_pio)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${PROJECT} pico_cyw43_arch_none)
endif()

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT})

# open serial port
#pico_enable_stdio_uart(${PROJECT} 1)
pico_enable_stdio_usb(${PROJECT} 1)
