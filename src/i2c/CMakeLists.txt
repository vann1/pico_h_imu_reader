
add_library(i2c_helpers INTERFACE)

target_include_directories(i2c_helpers INTERFACE 
${CMAKE_CURRENT_LIST_DIR}/include)
target_sources(i2c_helpers INTERFACE i2c_helpers.c)
