add_library(simulation
    "simulation.cc"
    "viewer.cc"
)

target_link_libraries(simulation PUBLIC
    bolt_base
    bolt_eigen_dense
    glfw
    mujoco::mujoco
)

target_include_directories(simulation PUBLIC "${PROJECT_SOURCE_DIR}/src")