# Value Formats
This file defines the formats for different value types.
All value types are stored as JSON objects containing a type tag followed by specialized elements. Each section below is named according to the value's type tag.

Ex.
```
{
  type: "type-tag",
  ...
}
```

Note: We represent rotations by quaternions with elements in [w, x, y, z] order.

Note: All current supported values are simple enough that we do not support encapsulation of values (i.e. value definitions do not depend on other value definitions)

### Rot2
* theta (double)

### Pose2
* x (double)
* y (double)
* theta (double)

### Rot3
* rotation (vector[4]: double[w, x, y, z])

### Pose3
* translation (vector[3]: double[x, y, z])
* rotation (vector[4]: double[w, x, y, z])

### Vector
* data (vector[?]: double[...])

### Point2
* x (double)
* y (double)

### Point3
* x (double)
* y (double)
* z (double)

### Unit3
* i (double)
* j (double)
* k (double)

### BearingRange (2D, Pose2)
* bearing (double)
* range (double)

### BearingRange (3D, Pose3)
* bearing (vector[3]: double)
* range (double)


# Values Format
We often have collections of values. Thus we define a type for this collection known as `Values`. A `Values` is a Map[uint64_t -> Value]. For example a `Values` may look like:

```
{
  23452352345: {
    "type": "Rot3"
    "rotation": [...]
  },
  ...
}
```