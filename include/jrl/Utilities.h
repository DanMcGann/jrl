/** @brief The utilities module provides implementation of generic algorithms
 * and used throughout the jrl library.
 */
#pragma once
#include <gtsam/nonlinear/Values.h>

#include <functional>
#include <vector>

namespace jrl {
namespace utils {

/** @brief Compute the Cartesian Product of vector of vectors
 * @param input: A vector of vectors over which to compute the cartesian product
 * @returns vector containing each combination from the input
 *
 * Ex. input [[a,b], [c], [d, e]]
 * will produce [[a,c,d], [a,c,e], [b,c,d], [v,c,e]]
 *
 * Cite: https://stackoverflow.com/questions/5279051/how-can-i-create-cartesian-product-of-vector-of-vectors
 */
template <typename T>
inline std::vector<std::vector<T>> cartesianProduct(const std::vector<std::vector<T>>& input);

/** @brief Filters values according to a predicate
 * @param values: The values to filter
 * @param pred: The filter predicate
 * @returns All values (key, val) for which the predicate evaluates to true
 */
inline gtsam::Values filter_values(const gtsam::Values& values,
                                   std::function<bool(const gtsam::Values::ConstKeyValuePair&)> pred) {
  gtsam::Values filtered;
  for (const auto& kvp : values) {
    if (pred(kvp)) filtered.insert(kvp.key, kvp.value);
  }
  return filtered;
}

/** @brief Filters values according to a type
 * @param values: The values to filter
 * @returns All values (key, val) of type T
 */
template <typename T>
inline gtsam::Values filter_types(const gtsam::Values& values);

}  // namespace utils
}  // namespace jrl

// Implementation
#include "jrl/Utilities-inl.h"