# Add sources for each plugin to be registered.
add_library(ESCDriver SHARED ESCDriver.cc)
set_property(TARGET ESCDriver PROPERTY CXX_STANDARD 17)
target_link_libraries(ESCDriver
  PUBLIC gz-plugin::gz-plugin
  PUBLIC gz-sim::gz-sim)

# Install the library
install(TARGETS ESCDriver
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib/gz-sim-10/plugins
    ARCHIVE DESTINATION lib/gz-sim-10/plugins)
