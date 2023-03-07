# CMake instructions to make the static lib

# Min CMake version
cmake_minimum_required(VERSION 3.20)

# First list the sources that shall be compiled
# Then Point CMake to the header files in the "include" directory
# Following this set the ESP32 component dependancies of the project
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
