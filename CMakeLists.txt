cmake_minimum_required(VERSION 3.16)
project(Hexapo-MuJoCo)
set(CMAKE_CXX_STANDARD 17)  # Required by MuJoCo 3.2.7

# Allow seamselss linking in a Conda environment
if(DEFINED ENV{CONDA_PREFIX})
    link_directories(AFTER $ENV{CONDA_PREFIX}/lib)
endif()

# DEPENDENCIES
# =============================================================================
include(FetchContent)

# MuJoCo si GLFW stáhne samo, pokud je potřeba
FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG        3.2.7
)
FetchContent_MakeAvailable(mujoco)

# Download Bolt
FetchContent_Declare(
    bolt
    GIT_REPOSITORY https://gitlab.fel.cvut.cz/valoudav/bolt.git
)
FetchContent_MakeAvailable(bolt)

# Změněno na HTTPS připojení
FetchContent_Declare(
    tgc
    GIT_REPOSITORY https://gitlab.fel.cvut.cz/crl-codebase/tgc-twist-gait-controller.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(tgc)
# =============================================================================

add_subdirectory(src)