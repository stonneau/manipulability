cmake_minimum_required(VERSION 2.6)
project(manipulability_app)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

set(MANIP_APP_ROOT "${PROJECT_SOURCE_DIR}")
set(MANIP_APP_INCLUDE "${PROJECT_SOURCE_DIR}/include/")


###############################################
find_package(Eigen3 REQUIRED)


add_subdirectory (src/manipulability_core)

