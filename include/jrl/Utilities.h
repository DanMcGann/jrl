#pragma once
/** @brief The utilities module provides implementation of generic algorithms
 * and used throughout the jrl library.
 */
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

}  // namespace utils
}  // namespace jrl

// Implementation
#include "jrl/Utilities-inl.h"