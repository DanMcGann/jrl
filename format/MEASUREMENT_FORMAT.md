# Measurement Formats
This file defines the formats for different measurement types.
All measurements types are stored as JSON objects containing a type tag followed by specialized elements. Each section below is named according to the measurements's type tag.

Ex.
```
{
  type: "type-tag",
  ...
}
```

Note: We represent measurement uncertainties by covariance matrices stored as vectors of the matrix's elements in row-major order.

Note: Measurements can support multiple types we explicitly enumerate the supported types below.

Note: We use a special case meta types (2D) and (3D) for measurements that are between a Pose2-Point2 and Pose3-Point3 respectively.

### PriorFactor\<TYPE\>
* \<TYPE\> = Pose2, Pose3, Point2, Point3
* Elements:
  * key (uint64)
  * prior (Value of type: TYPE)
  * covariance (vector[?]: double[...])

### BetweenFactor\<TYPE\>
* \<TYPE\> = Pose2, Pose3, Point2, Point3
* Elements:
  * key1 (uint64)
  * key2 (uint64)
  * measurement (Value of type: TYPE)
  * covariance (vector[?]: double[...])

### RangeFactor\<TYPE\>
* \<TYPE\> = Pose2, Pose3, 2D, and 3D
* Elements:
  * key1 (uint64)
  * key2 (uint64)
  * measurement (double)
  * covariance (vector[1]: double)

### BearingRangeFactor\<TYPE\>
* \<TYPE\> = Pose2, Pose3, 2D, and 3D
* Elements:
  * key1 (uint64)
  * key2 (uint64)
  * measurement BearingRange (\<TYPE\>)
  * covariance (vector[?]: double)