file(GLOB_RECURSE files "*.c")

add_library(read_ops ${files})

target_include_directories(read_ops PUBLIC .)

target_link_libraries(read_ops
PRIVATE
ism330dlc_registers
<<<<<<< HEAD
ism330dlc
utils
=======
<<<<<<< HEAD
ism330dlc
utils
=======
>>>>>>> fc3668f (Fresh start)
>>>>>>> d205c2c (?)
PUBLIC
Fusion
)

