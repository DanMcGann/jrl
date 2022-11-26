#include <sstream>

#include "jrl/Dataset.h"

namespace jrl {
/**********************************************************************************************************************/
Results::Results(const std::string& name, std::vector<char>& robots, std::map<char, TypedValues>& solutions)
    : name(name), robots(robots), robot_solutions(solutions) {}

/**********************************************************************************************************************/
Results::Results(const std::string& name, std::vector<char>& robots) : name(name), robots(robots) {}
}  // namespace jrl
