cmake_minimum_required(VERSION 3.16)
project(Hexapo-MuJoCo)
set(CMAKE_CXX_STANDARD 17)  # Required by MuJoCo 3.2.7
add_compile_options(-Wno-dangling-reference)

# Allow seamselss linking in a Conda environment
if(DEFINED ENV{CONDA_PREFIX})
    link_directories(AFTER $ENV{CONDA_PREFIX}/lib)
endif()

include(FetchContent)

# Download MuJoCo
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

# Download ProxSuite
FetchContent_Declare(
    proxsuite
    GIT_REPOSITORY https://github.com/Simple-Robotics/proxsuite.git
    GIT_TAG        v0.8.2
)
FetchContent_MakeAvailable(proxsuite)

# Find Pinocchio
find_package(pinocchio REQUIRED)

add_subdirectory(src)