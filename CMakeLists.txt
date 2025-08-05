
cmake_minimum_required(VERSION 3.28)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(ism330dlc_reader)
add_subdirectory(src)
# Disable automatic picotool installation
set(PICO_NO_PICOTOOL 1)

message(STATUS "PICO_SDK_PATH is: ${PICO_SDK_PATH}")

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(ism330dlc_reader
    ism330dlc_reader.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(
    ism330dlc_reader
    pico_stdlib
    hardware_i2c
    ism330dlc
)

# enable usb output, disable uart output
pico_enable_stdio_usb(ism330dlc_reader 1)
pico_enable_stdio_uart(ism330dlc_reader 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(ism330dlc_reader)
