#include "perception_pkg/assignment_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception_pkg {

std::vector<int> solve_assignment(
    const std::vector<std::vector<double>>& costs,
    double unmatched_cost)
{
  if (!std::isfinite(unmatched_cost)) {
    throw std::invalid_argument("unmatched_cost must be finite");
  }
  const size_t row_count = costs.size();
  const size_t column_count = row_count == 0 ? 0 : costs.front().size();
  for (const auto& row : costs) {
    if (row.size() != column_count) {
      throw std::invalid_argument("assignment cost matrix must be rectangular");
    }
  }

  const size_t size = std::max(row_count, column_count);
  if (size == 0) {
    return {};
  }

  std::vector<std::vector<double>> square(
      size, std::vector<double>(size, unmatched_cost));
  for (size_t row = 0; row < row_count; ++row) {
    for (size_t column = 0; column < column_count; ++column) {
      square[row][column] = costs[row][column];
    }
  }

  // Hungarian algorithm for a square minimum-cost assignment, using 1-based
  // indexing internally. Stable input order makes equal-cost ties deterministic.
  std::vector<double> row_potential(size + 1, 0.0);
  std::vector<double> column_potential(size + 1, 0.0);
  std::vector<size_t> column_match(size + 1, 0);
  std::vector<size_t> previous_column(size + 1, 0);
  for (size_t row = 1; row <= size; ++row) {
    column_match[0] = row;
    size_t current_column = 0;
    std::vector<double> minimum(
        size + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(size + 1, false);
    do {
      used[current_column] = true;
      const size_t current_row = column_match[current_column];
      double delta = std::numeric_limits<double>::infinity();
      size_t next_column = 0;
      for (size_t column = 1; column <= size; ++column) {
        if (used[column]) {
          continue;
        }
        const double reduced_cost = square[current_row - 1][column - 1] -
            row_potential[current_row] - column_potential[column];
        if (reduced_cost < minimum[column]) {
          minimum[column] = reduced_cost;
          previous_column[column] = current_column;
        }
        if (minimum[column] < delta) {
          delta = minimum[column];
          next_column = column;
        }
      }
      for (size_t column = 0; column <= size; ++column) {
        if (used[column]) {
          row_potential[column_match[column]] += delta;
          column_potential[column] -= delta;
        } else {
          minimum[column] -= delta;
        }
      }
      current_column = next_column;
    } while (column_match[current_column] != 0);

    do {
      const size_t previous = previous_column[current_column];
      column_match[current_column] = column_match[previous];
      current_column = previous;
    } while (current_column != 0);
  }

  std::vector<int> assignment(row_count, -1);
  for (size_t column = 1; column <= size; ++column) {
    const size_t row = column_match[column];
    if (row > 0 && row <= row_count && column <= column_count) {
      assignment[row - 1] = static_cast<int>(column - 1);
    }
  }
  return assignment;
}

}  // namespace perception_pkg
