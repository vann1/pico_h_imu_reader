add_library(ism330dlc INTERFACE)
target_include_directories(ism330dlc INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/include)
 
target_link_libraries(
	ism330dlc INTERFACE 
	ism330dlc_registers
	ism330dlc_config
	read_ops
	utils	
)
target_sources(ism330dlc INTERFACE ism330dlc.c)
