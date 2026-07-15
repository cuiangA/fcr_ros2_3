#pragma once

#include <vector>

namespace perception_pkg {

/**
 * Solve a rectangular minimum-cost assignment with the Hungarian algorithm.
 * The returned vector has one entry per input row; -1 means unmatched.
 */
std::vector<int> solve_assignment(
    const std::vector<std::vector<double>>& costs,
    double unmatched_cost = 1.0);

}  // namespace perception_pkg
