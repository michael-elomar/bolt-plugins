
# Add sources for each plugin to be registered.
add_library(Example SHARED Example.cc)
set_property(TARGET Example PROPERTY CXX_STANDARD 17)
target_link_libraries(Example
  PUBLIC gz-plugin::gz-plugin
  PUBLIC gz-sim::gz-sim)

# Install the library
install(TARGETS Example
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib/gz-sim-10/plugins
    ARCHIVE DESTINATION lib/gz-sim-10/plugins)
