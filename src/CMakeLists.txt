add_subdirectory(simulation)
add_subdirectory(tgc)

# Define main executable
add_executable(run "main.cc")

target_link_libraries(run PUBLIC
    simulation
    tgc
    bolt_parsers_config
)

set_target_properties(run PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
