
add_library(i2c_helpers INTERFACE)

target_include_directories(i2c_helpers INTERFACE 
${CMAKE_CURRENT_LIST_DIR}/include)
target_sources(i2c_helpers INTERFACE i2c_helpers.c)

target_link_libraries(i2c_helpers
INTERFACE
pico_stdlib
hardware_i2c
ism330dlc
)