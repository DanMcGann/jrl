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
}
```

Where an entry represents a set of measurements taken at a specific time and has the format:
```
{
  # uint64_t - The timestamp at witch the contained measurements were observed
  "stamp": 1234

  # List-of-Measurement - The measurements taken at timestamp stamp
  "measurements": [...]

  # (Optional) Map[uint64_t -> bool] - Mapping of factor indicies (in "measurements") to their outlier status
  # If a factor index appears as a key it is possibly an outlier measurement (e.g. loop-closures)
  # The value for each key indicates if the meaurement is actually an outlier
  "potential_outlier_statuses": {"2": false, "6": true, ...}
}
```

For the definition of Values format see `VALUE_FORMAT.md`.
For the definition of Measurement formats see `MEASUREMENT_FORMAT.md`.