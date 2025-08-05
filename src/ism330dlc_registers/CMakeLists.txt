message(STATUS "ism330dlc registers have been loaded")
add_library(ism330dlc_registers INTERFACE)
target_include_directories(ism330dlc_registers INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/include)
