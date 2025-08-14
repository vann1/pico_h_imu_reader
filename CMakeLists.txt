
cmake_minimum_required(VERSION 3.28)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(ism330dlc_reader)
# Disable automatic picotool installation
# set(PICO_NO_PICOTOOL 1)

message(STATUS "PICO_SDK_PATH is: ${PICO_SDK_PATH}")

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(src)

# rest of your project
add_executable(main
    main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(
    main
    pico_stdlib
    hardware_i2c
    ism330dlc
    utils
    i2c_helpers
    Fusion
    sh2_wrapper
    read_ops
)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(main)
