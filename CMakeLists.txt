# CMake instructions to make the static lib

# Min CMake version
cmake_minimum_required(VERSION 3.20)

# idf_component_register(REQUIRES 
# 						esp_system
# 						nvs_flash
# 						wifi_provisioning
# 						)				 

idf_component_register(SRCS "WifiModule.cpp"
							"I2SModule.cpp"
						INCLUDE_DIRS "./include"
						REQUIRES 
						wifi_provisioning
						nvs_flash
						esp_system
						driver
						Chunk_Types 
						Base_Module
						)
