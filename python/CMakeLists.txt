message(STATUS "================  PYTHON BINDINGS ENABLED. BUILDING BINDINGS  ======================")

# Find Python Interpreter
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# Fetch pybind to build the python bindings
find_package(pybind11 2.11 QUIET)
if(NOT pybind11_FOUND)
    MESSAGE(STATUS "No compatible version of PyBind found. Retrieving via fetch content.")
    include(FetchContent)
    FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.11.0
    )
    FetchContent_MakeAvailable(pybind11)
endif()

# Python Bindings
file(GLOB_RECURSE pybind_srcs "*.cpp" "*.h")
pybind11_add_module(jrl_python ${pybind_srcs})
target_link_libraries(jrl_python PRIVATE jrl gtsam nlohmann_json::nlohmann_json)

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
configure_file(${PROJECT_SOURCE_DIR}/cmake/setup.py.in ${JRL_PYTHON_BUILD_DIRECTORY}/setup.py)

# Create an actual python module in the build structure. Technically this is the module that gets installed
# It simply imports everything from the .so module built by pybind11
file(GENERATE OUTPUT ${JRL_PYTHON_BUILD_DIRECTORY}/jrl/__init__.py CONTENT "from jrl.jrl_python import *")

# Setup the install command. Simply delegates to pip (should work with environmets)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
add_custom_target(jrl-python-install
        COMMAND ${Python_EXECUTABLE} -m pip install .
        DEPENDS jrl_python
        WORKING_DIRECTORY ${JRL_PYTHON_BUILD_DIRECTORY})

# Setup uninstall command. This can also be done with pip
add_custom_target(jrl-python-uninstall
        COMMAND ${Python_EXECUTABLE} -m pip uninstall jrl
        WORKING_DIRECTORY ${JRL_PYTHON_BUILD_DIRECTORY})
