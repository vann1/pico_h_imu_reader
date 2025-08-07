message(STATUS "Utils modules has been loaded")

add_library(utils INTERFACE)

target_include_directories(utils INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_sources(utils INTERFACE bit_ops.c)