file(GLOB_RECURSE files "*.c")

add_library(read_ops ${files})

target_include_directories(read_ops PUBLIC .)

target_link_libraries(read_ops
PRIVATE
ism330dlc_registers
ism330dlc
utils
PUBLIC
Fusion
)

