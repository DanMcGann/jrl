cmake_minimum_required(VERSION 3.0.0)
project(jrl VERSION 0.0.2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Dependencies
find_package(GTSAM REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# BUILD: JRL Library
file(GLOB_RECURSE jrl_srcs "src/*.cpp")
file(GLOB_RECURSE jrl_headers "include/*.h")
add_library(jrl ${jrl_srcs} ${jrl_headers})
target_link_libraries(jrl gtsam nlohmann_json::nlohmann_json)
target_include_directories(jrl PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)

# INSTALL: JRL Library
install(FILES ${jrl_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/jrl)
install(
	TARGETS jrl
    EXPORT jrl-exports
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
include(cmake/HandleUninstall.cmake) # for "make jrl-uninstall"


# Build Tests for JRL if configured
option(JRL_BUILD_TESTS "Build unit tests" OFF)
if (${JRL_BUILD_TESTS})
    add_subdirectory(tests)
endif()

# Build Python Bindings for JRL if configured
option(JRL_BUILD_PYTHON "Build python bindings" OFF)
if (${JRL_BUILD_PYTHON})
    add_subdirectory(python)
endif()