#include "jrl/Results.h"

#include <sstream>

namespace jrl {
/**********************************************************************************************************************/
Results::Results(const std::string& dataset_name, const std::string& method_name, std::vector<char>& robots,
                 std::map<char, TypedValues>& solutions)
    : dataset_name(dataset_name), method_name(method_name), robots(robots), robot_solutions(solutions) {}

/**********************************************************************************************************************/
Results::Results(const std::string& dataset_name, const std::string& method_name, std::vector<char>& robots)
    : dataset_name(dataset_name), method_name(method_name), robots(robots) {}
}  // namespace jrl
