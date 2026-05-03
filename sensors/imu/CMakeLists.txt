# Add sources for each plugin to be registered.
add_library(ImuSensor SHARED Imu.cc)
set_property(TARGET ImuSensor PROPERTY CXX_STANDARD 17)
target_link_libraries(ImuSensor
  PUBLIC gz-sensors::gz-sensors gz-rendering::gz-rendering)

# Install the library
install(TARGETS ImuSensor
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

# build system plugin
# Add sources for each plugin to be registered.
add_library(ImuSystem SHARED ImuSystem.cc)
set_property(TARGET ImuSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(ImuSystem
  PUBLIC ImuSensor
  PUBLIC gz-plugin::gz-plugin
  PUBLIC gz-sensors::gz-sensors
  PUBLIC gz-sim::gz-sim)

# Install the library
install(TARGETS ImuSystem
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib/gz-sim-10/plugins
    ARCHIVE DESTINATION lib/gz-sim-10/plugins)
