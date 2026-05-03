# Add sources for each plugin to be registered.
add_library(Barometer SHARED Barometer.cc)
set_property(TARGET Barometer PROPERTY CXX_STANDARD 17)
target_link_libraries(Barometer
  PUBLIC gz-sensors::gz-sensors gz-rendering::gz-rendering)

# Install the library
install(TARGETS Barometer
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

# build system plugin
# Add sources for each plugin to be registered.
add_library(BarometerSytem SHARED BarometerSystem.cc)
set_property(TARGET BarometerSytem PROPERTY CXX_STANDARD 17)
target_link_libraries(BarometerSytem
  PUBLIC Barometer
  PUBLIC gz-plugin::gz-plugin
  PUBLIC gz-sensors::gz-sensors
  PUBLIC gz-sim::gz-sim)

# Install the library
install(TARGETS BarometerSytem
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib/gz-sim-10/plugins
    ARCHIVE DESTINATION lib/gz-sim-10/plugins)
