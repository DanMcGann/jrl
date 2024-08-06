# Dataset Format
This file defines the format of JRL dataset files. These files are stored with `.jrl` (JSON Robot Log) extensions.

A dataset is a JSON object that contains the following components:
```
{
  # string - The name of the dataset
  "name": "name",

  # List-of-char - The ids of the robots in the dataset
  "robots: ['a' ...]

  # Map[char -> List-of-Entry] - A list of entries containing the measurements taken by each robot in the dataset
  "measurements": {"a": [...], "b": [...]}

  # (Optional) Map[char -> Values] - The groundtruth solution for the values for each robot in the dataset
  "groundtruth": {"a": {...}, "b": {...}}

  # (Optional) Map[char -> Values] - The initial estimate for the values for each robot in the dataset (used for batch algorithms)
  "initialization": {"a": {...}, "b": {...}}

  # (Optional) [List-of-FactorId] - The set of factors that are potentially outliers (i.e. Loop Closures)
  # If not included all measurements are assumed to be known inliers
  "potential_outlier_factors": [ [0, 2], [4, 0] ...]

  # (Optional) [List-of-FactorId] - The set of ground truth outlier factors
  "outlier_factors": [[4, 0], ...]
}
```

Where an entry represents a set of measurements taken at a specific time and has the format:
```
{
  # uint64_t - The timestamp at witch the contained measurements were observed
  "stamp": 1234

  # List-of-Measurement - The measurements taken at timestamp stamp
  "measurements": [...]
}
```

And a FactorId = `Pair[uint64_t, uint64_t]` represents a unique identifier of a measurement by:
  - "Entry Index" - The index of the entry to which the factor belongs
  - "Measurement Index" - The index of the measurement within the entry at the Entry Index

For the definition of Values format see `VALUE_FORMAT.md`.
For the definition of Measurement formats see `MEASUREMENT_FORMAT.md`.