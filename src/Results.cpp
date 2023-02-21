#include "jrl/Results.h"

#include <sstream>

namespace jrl {
/**********************************************************************************************************************/
Results::Results(std::string dataset_name, std::string method_name, std::vector<char> robots,
                 std::map<char, TypedValues> solutions)
    : dataset_name(dataset_name), method_name(method_name), robots(robots), robot_solutions(solutions) {}

/**********************************************************************************************************************/
Results::Results(std::string dataset_name, std::string method_name, std::vector<char> robots)
    : dataset_name(dataset_name), method_name(method_name), robots(robots) {}
}  // namespace jrl
