# CMake instructions to make the static lib

# Min CMake version
cmake_minimum_required(VERSION 3.20)

idf_component_register(SRCS "WifiModule.cpp"
						INCLUDE_DIRS "./include"
						REQUIRES 
						wifi_provisioning 
						Chunk_Types 
						Base_Module
						)


# What to add to library
# add_library(SC_Generic_Module_Types STATIC 
# 			"RouterModule.h"
# 			"RouterModule.cpp"
# 			"SessionProcModule.h"
# 			"SessionProcModule.cpp"
# 			"WavAccumulator.h"
# 			"WavAccumulator.cpp"
# 			"SimulatorModule.h"
# 			"SimulatorModule.cpp"
# 			)


# Library Procedure
# 1. Find the library of interest
# 2. Include the source files so they can be referenced
# 3. Inform compiler where to find library such that it may be linked

# # Adding Chunk Types libs to modules
# find_library(SC_Chunk_Types PUBLIC $ENV{SC_LIB_DIR}/Lib_Chunk_Types/CPP_Chunk_Types/Build_ESP32)
# include_directories(SC_Chunk_Types INTERFACE $ENV{SC_LIB_DIR}/Lib_Chunk_Types/Chunk_Types)
# target_link_libraries(SC_Generic_Module_Types PUBLIC SC_Chunk_Types)

# # Adding Base Module libs to Generic Modules
# find_library(SC_Base_Module_Types PUBLIC $ENV{SC_LIB_DIR}/Lib_Base_Module/BaseModule/Build_ESP32)
# include_directories(SC_Base_Module_Types INTERFACE $ENV{SC_LIB_DIR}/Lib_Base_Module/BaseModule)
# target_link_libraries(SC_Generic_Module_Types PUBLIC SC_Base_Module_Types)