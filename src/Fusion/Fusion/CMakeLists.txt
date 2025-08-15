file(GLOB_RECURSE files "*.c")

add_library(Fusion ${files})

target_include_directories(Fusion INTERFACE .)

target_link_libraries(Fusion
PRIVATE 
pico_stdlib
hardware_i2c
hardware_rtc
PUBLIC
i2c_helpers
 )

# if(UNIX AND NOT APPLE)
#     target_link_libraries(Fusion m) # link math library for Linux
# endif()
