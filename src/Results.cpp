#include "jrl/Results.h"

#include <sstream>

namespace jrl {
/**********************************************************************************************************************/
Results::Results(std::string dataset_name, std::string method_name, std::vector<char> robots,
                 std::map<char, TypedValues> solutions,
                 boost::optional<std::map<char, std::set<FactorId>>> robot_outliers)
    : dataset_name(dataset_name),
      method_name(method_name),
      robots(robots),
      robot_solutions(solutions),
      robot_outliers(robot_outliers) {}

/**********************************************************************************************************************/
Results::Results(std::string dataset_name, std::string method_name, std::vector<char> robots)
    : dataset_name(dataset_name), method_name(method_name), robots(robots) {}
}  // namespace jrl
