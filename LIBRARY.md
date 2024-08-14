# JRL Library

In addition to defining the platform / language agnostic JRL formats this package also provides a library to make it easy to work with JRL data in C++ and Python. 

## Modules 
The library contains the following Main Modules:
* `Dataset` - Container for JRL Datasets
* `Results` - Container for JRR Results
* `Metrics` - Provides container for JRM metrics as well as functions to compute these metrics from Results and Datasets.
* `Writer` -  Provides functionality to serialize (optionally compress) and write Dataset, Result, and Metric files.
  * Utilizes functionally in `IOMeasurements` and `IOValues`
* `Parser` - Provides functionality to read and parse (optionally compressed) Dataset, Result and Metric files.
  * Utilizes functionally in `IOMeasurements` and `IOValues`

The library also provides some additional helper modules:
* `Types` - Helper to various modules, provides definitions of types used throughout the library
* `Alignment` - Helper to Metrics module, provides functionality to perform Umeyama alignment between trajectories.
* `IOMesaurements` - Helper to Parser/Writer, provides functionality to serialize/parse the various measurement types.
* `IOValues` - Helper to Parser/Writer, provides functionality to serialize/parse the various value types.
* `Utilities` - Helper to various modules, provides generic algorithm implementations.
* `Initialization` - Helper for user scripts, provides functionality to compute the initialization of new variables in an entry given the solution to the problem defined up to and including the previous entry.

## Python Bindings
Python bindings are provided for all modules in `python/`. Python bindings follow the same naming conventions as the C++ implementation, with the caveat that template types are realized by appending the type to the class name. For example given a C++ class `jrl::Class<Type>` the corresponding python binding will be `jrl.ClassType`. 

Enable building the python bindings with `JRL_BUILD_PYTHON` CMake option. The bindings can be installed with `make jrl-python-install` and uninstalled with `jrl-python-uninstall`. These commands delegate to pip under the hood and can work with virtual environments by setting the CMake variable `PYTHON_EXECUTABLE` to the proper path for your environment (find by running `which python` when the environment is active). For this to work GTSAM must also be installed in the environment which can be done in a similar manner to the method described here for JRL.

## Tests
Some modules are tested via unit tests in `tests/`. Enable building tests with `JRL_BUILD_TESTS` CMake option, and run tests with `make jrl-test`. We of course are are always looking for help to build out our test suite, and appreciate all contributions to do so!

## Library Design Decisions
The following document some implementation / design decisions so that users can understand some of our rationale.

#### Optional
We use `boost::optional` over the c++17 supported `std::optional` as that is what is used by GTSAM.

#### Containers
We use `stl` containers over GTSAM "fast" containers to allow implicit conversion via pybind11's `stl.h`

#### Dependencies (Ubuntu 20.04)
JRL relies heavily on GTSAM for its geometry, factor, and container types. GTSAM should be installed on the machine or a local build of GTSAM should be linked by setting `GTSAM_DIR` and `GTSAM_INCLUDE_DIR` CMake options. 

Since C++ does not have a native support for parsing JSON we use a 3rd party library for JSON support. We specifically choose [nlohmann-json](https://github.com/nlohmann/json) because of its ease of use, quality design + implementation, and support across platforms.
* `sudo apt-get install nlohmann-json3-dev`

Tests (enabled with `JRL_BUILD_TESTS`) require the GoogleTest framework. This is automatically included with FetchContent if tests are enabled.

Python Bindings require the Pybind11 library. This is automatically included with FetchContent if bindings are enabled.

#### Compression
The Parser and Writer classes have options to enable [cbor](https://cbor.io/) compression to decrease the resulting file size. This can be incredibly important if you are saving intermediate results of an algorithm. This appears to typically reduce result file size by half!

For portability of datasets, we recommend that you do not compress datasets as you typically have only a few of them. Compression is most useful for limiting storage requirements iterative/incremental result files. 

## Install Instructions
```
mkdir build
cd build
cmake .. -DJRL_BUILD_PYTHON=On -DJRL_BUILD_TESTS=On # Building Python and Tests are Optional
make

# To install the c++ library
sudo make install

# To install the python module
sudo make jrl-python-install
```

## Uninstall Instructions
```
cd build
sudo make jrl-uninstall # to uninstall the c++ library
sudo make jrl-python-uninstall # to uninstall the python module
```

# Issues
If you find any bugs / issues with this library please submit a bug report via the github issue tracker. If the issue pertains to an implementation bug, please include a unit tests that demonstrates the issue.