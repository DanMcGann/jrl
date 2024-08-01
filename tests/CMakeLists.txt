message(STATUS "================  TESTS ENABLED. SETTING UP TESTS  ======================")

# Setup gtest
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        origin/main
)
option(INSTALL_GTEST OFF)
FetchContent_MakeAvailable(googletest)
enable_testing() # enable ctest

# Make test target
file(GLOB_RECURSE tests_srcs *.cpp)
add_executable(jrl-tests ${tests_srcs})
target_link_libraries(jrl-tests PUBLIC jrl gtsam gtest)
add_test(NAME jrl-tests COMMAND jrl-tests)

# Make runnable from "make check-jrl" or "make test-jrl"
add_custom_target(jrl-check COMMAND jrl-tests)
add_custom_target(jrl-test COMMAND jrl-tests)