# Add sources for each plugin to be registered.
add_library(GnssSensor SHARED Gnss.cc)
set_property(TARGET GnssSensor PROPERTY CXX_STANDARD 17)
target_link_libraries(GnssSensor
  PUBLIC gz-sensors::gz-sensors gz-rendering::gz-rendering)

# Install the library
install(TARGETS GnssSensor
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)


# build system plugin

# Add sources for each plugin to be registered.
add_library(GnssSystem SHARED GnssSystem.cc)
set_property(TARGET GnssSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(GnssSystem
  PUBLIC GnssSensor
  PUBLIC gz-plugin::gz-plugin
  PUBLIC gz-sensors::gz-sensors
  PUBLIC gz-sim::gz-sim)

# Install the library
install(TARGETS GnssSystem
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib/gz-sim-10/plugins
    ARCHIVE DESTINATION lib/gz-sim-10/plugins)
