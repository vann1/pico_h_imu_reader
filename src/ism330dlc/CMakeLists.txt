add_library(ism330dlc INTERFACE)
target_link_libraries(
	ism330dlc INTERFACE 
	ism330dlc_registers
	ism330dlc_config
)
target_sources(ism330dlc INTERFACE ism330dlc.c)
