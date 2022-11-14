# List of supported values and their formats

### General Notes
We follow gtsam conventions for data structures therefore...
* Quaternions follow form: [w, x, y, z]
* Matricies are in row-major order


#### Pose2
* x (double)
* y (double)
* theta (double)

#### Pose3
* translation (vector: double)
* rotation (vector: quaternion)