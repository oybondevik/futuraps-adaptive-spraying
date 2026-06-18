#pragma once
#include <cmath>
#include <utility>

namespace futuraps {

struct GridSpec {
  double min_x, max_x, min_z, max_z;
  double cell_size;
  double min_y, max_y;
  double origin_x, origin_z;
  unsigned width, height;
};

inline bool in_bounds(const GridSpec& g, double x, double z) {
  return x >= g.min_x && x < g.max_x && z >= g.min_z && z < g.max_z;
}

inline std::pair<int,int> to_index(const GridSpec& g, double x, double z) {
  int i = static_cast<int>(std::floor((x - g.origin_x) / g.cell_size));
  int j = static_cast<int>(std::floor((z - g.origin_z) / g.cell_size));
  return {i, j};
}

inline size_t flat_index(const GridSpec& g, int i, int j) {
  return static_cast<size_t>(i) * g.height + static_cast<size_t>(j);
}

} // namespace futuraps
