#include "jrl/Utilities.h"

namespace jrl {
namespace utils {


/**********************************************************************************************************************/
template <typename T>
inline std::vector<std::vector<T>> cartesianProduct(const std::vector<std::vector<T>>& input) {
  std::vector<std::vector<T>> s = {{}};
  for (const auto& u : input) {
    std::vector<std::vector<T>> r;
    for (const auto& x : s) {
      for (const auto y : u) {
        r.push_back(x);
        r.back().push_back(y);
      }
    }
    s = std::move(r);
  }
  return s;
}


}  // namespace utils
}  // namespace jrl