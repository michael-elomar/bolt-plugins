cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(BoltSensors VERSION 0.0.0)

#============================================================================
# Find gz-cmake
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-cmake REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin REQUIRED COMPONENTS loader register)

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport REQUIRED COMPONENTS log parameters)

#--------------------------------------
# Find gz-sensors
gz_find_package(gz-sensors REQUIRED)

#--------------------------------------
# Find gz-msgs
gz_find_package(gz-msgs REQUIRED)

#--------------------------------------
# Find gz-rendering
gz_find_package(gz-rendering REQUIRED)

# Find gz-msgs
gz_find_package(gz-sim REQUIRED)


include_directories(${GZ-PLUGIN_INCLUDE_DIRS})
link_directories(${GZ-PLUGIN_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GZ-PLUGIN_CXX_FLAGS}")

add_subdirectory(sensors)
add_subdirectory(actuators)

add_subdirectory(example)

# find_package(Protobuf REQUIRED)
#
# set(CMAKE_C_STANDARD 23)
# set(CMAKE_C_STANDARD_REQUIRED ON)
#
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
#
# # compile protobuf files
# add_subdirectory(proto)
# get_property(PROTO_CC_HEADERS GLOBAL PROPERTY PROTO_CC_HEADERS)
# list(APPEND INCLUDE ${PROTO_CC_HEADERS})
#
# # compile gnss plugin
# add_subdirectory(gnss)


