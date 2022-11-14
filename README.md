# JSON Robot Log

This Document defines the format of the JSON Robot Log file format. The file format's purpose is to provide a general purpose file format for SLAM datasets.

JRL format seeks to improve existing file formats like g2o, or the much less popular toro by supporting...
- Explicit temporal information to evaluate real-time performance
- Encode multi-robot datasets
- Encode Ground-Truth explicitly (and in the same file so it cannot get lost)
- (Optionally) Encode (potentially, multiple) initialization values for batch methods



## Design Decisions / Conventions

#### Symbols
We seek to abide by the GTSAM symbol convention for ease of use with GTSAM as a backend. This, however limits us as GTSAM Symbols allow for only a single character making it impossible to encode both a robot identifier and semantic variable type identifier. By convention JRL uses the symbol as a robot identifier, and differentiates variable types by partitioning the integer space of the key.

* Variable Type
    * Pose (Point2/3 or Pose2/2 depending on linearity of problem) : `0 - 999,999,999` (`0-Billions`)
    * Landmark (Point2/3) : `1,000,000,000` - `1,999,999,999` (`1-Billions`)
    * Linear Velocities (Vector): `2-Billions`
    * Angular Velocities (Vector): `3-Billions`
    * IMU Bias Terms: `4-Billions`
    * ... TODO

Note one billion was selected as the partition size arbitrarily, but we figured it would be pretty unlikely to have a single dataset with over a BILLION poses, or landmarks etc. Additionally, even with this partition size as gtsam uses a `uint64_t` it still allows for `18446744073` unique variable types which is more than enough for any practical use. 


### Optional
We use `boost::optional` over the c++17 supported `std::optional` as that is what is used by GTSAM.


### Containers
We use `stl` containers over GTSAM "fast" containers to allow implicit conversion via pybind11's `stl.h`



## Dependencies (Ubuntu 20.04)
* `sudo apt-get install nlohmann-json3-dev`
