# Result Format
When evaluating SLAM algorithms we often need to save out results. To provide a unified format for such result files we define a simple result file format here:

```
{
  # string - The name of the dataset that these are results for
  "dataset_name": "my dataset"
  
  # string - The name of the method/algorithm that produced these results
  "method_name": "my algorithm"

  # List-of-char - The ids for all robots in the dataset 
  "robots": ["a" ...]

  # Map[char -> Values] - The computed results for all values of all robots in the dataset
  "solutions": {"a": {...}, ...}
}
```
Result files are saved with `.jrr` (JSON Robot Results) extensions.
For a definition of `Values` format see `VALUE_FORMAT.md`