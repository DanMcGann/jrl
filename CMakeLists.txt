project(jrl CXX C)
cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(GTSAM REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/pybind11)

# JRL Library
file(GLOB_RECURSE jrl_srcs "src/*.cpp" "include/*.h")
add_library(jrl ${jrl_srcs})
target_link_libraries(jrl gtsam nlohmann_json::nlohmann_json)
target_include_directories(jrl PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")


# Python Bindings
file(GLOB_RECURSE pybind_srcs "python/*")
pybind11_add_module(jrl_python ${pybind_srcs})
target_link_libraries(jrl_python PRIVATE jrl gtsam nlohmann_json::nlohmann_json)
target_include_directories(jrl_python PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

