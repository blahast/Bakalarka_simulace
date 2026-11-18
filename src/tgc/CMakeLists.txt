add_library(tgc STATIC
    "twist_gait.cc"
    "twist_gait_positional.cc"
)

target_link_libraries(tgc PUBLIC
    bolt_base
    bolt_eigen_dense
    bolt_kinematics
    yaml-cpp
    proxsuite::proxsuite
)

target_include_directories(tgc PUBLIC "${PROJECT_SOURCE_DIR}/src")