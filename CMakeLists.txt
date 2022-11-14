project(robust-inference CXX C)
cmake_minimum_required(VERSION 2.8.3)

find_package(GTSAM REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# JRL Library
file(GLOB_RECURSE jrl_srcs "*.cpp" "*.h")
add_library(jrl ${jrl_srcs})
target_link_libraries(jrl PRIVATE gtsam nlohmann_json::nlohmann_json)
target_include_directories(jrl PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")


# Python Bindings
pybind11_add_module(jrl_python python/bindings.cpp)
target_link_libraries(jrl_python jrl pybind11)

