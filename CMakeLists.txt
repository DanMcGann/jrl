project(jrl CXX C)
cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(GTSAM REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/pybind11)

# Custom Cmake
include(cmake/HandleUninstall.cmake)        # for "make uninstall"

# BUILD: JRL Library
file(GLOB_RECURSE jrl_srcs "src/*.cpp")
file(GLOB_RECURSE jrl_headers "include/*.h")
add_library(jrl ${jrl_srcs} ${jrl_headers})
target_link_libraries(jrl gtsam nlohmann_json::nlohmann_json)
target_include_directories(jrl PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)

# INSTALL JRL Library
install(FILES ${jrl_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/jrl)
install(
	TARGETS jrl
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Python Bindings
file(GLOB_RECURSE pybind_srcs "python/*")
pybind11_add_module(jrl_python ${pybind_srcs})
target_link_libraries(jrl_python PRIVATE jrl gtsam nlohmann_json::nlohmann_json)
target_include_directories(jrl_python PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")



set(JRL_PYTHON_BUILD_DIRECTORY ${PROJECT_BINARY_DIR}/python)
set_target_properties(jrl_python PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    OUTPUT_NAME "jrl_python"
    LIBRARY_OUTPUT_DIRECTORY "${JRL_PYTHON_BUILD_DIRECTORY}/jrl"
    DEBUG_POSTFIX "" # Otherwise you will have a wrong name
    RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
    )

# Install Python
# Generate the setup.py file in the correct directory. This will be used to install the python module.
configure_file(${PROJECT_SOURCE_DIR}/cmake/setup.py.in
                ${JRL_PYTHON_BUILD_DIRECTORY}/setup.py)
# Create an actual python module in the build structure. Technically this is the module that gets installed
# It simply imports everything from the .so module built by pybind11
file(GENERATE OUTPUT ${JRL_PYTHON_BUILD_DIRECTORY}/jrl/__init__.py CONTENT "from jrl.jrl_python import *")

# Setup the install command. Simply delegates to pip (should work with environmets)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
add_custom_target(python-install
        COMMAND ${Python_EXECUTABLE} -m pip install .
        DEPENDS jrl_python
        WORKING_DIRECTORY ${JRL_PYTHON_BUILD_DIRECTORY})

# Setup uninstall command. This can also be done with pip
add_custom_target(python-uninstall
        COMMAND pip uninstall jrl
        WORKING_DIRECTORY ${JRL_PYTHON_BUILD_DIRECTORY})
