#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cstdint>

namespace futuraps {

struct GridReq {
  float s;                 // cell_size
  uint32_t rows, cols;
  float x0, z0;            // grid origin (forward, height)
  float yL, yR;            // left/right limits (>=0)
  int8_t side;             // 0 both, 1 left, -1 right
  float p_front;           // 0..0.5 (front percentile of |y|)
  uint32_t min_pts;        // min points per cell
};

struct CellOut {
  bool     found = false;
  float    x = 0.f, y = 0.f, z = 0.f, dist = 0.f;
  uint32_t count = 0;
  float    confidence = 0.f;
};

// Compute closest-to-XZ-plane (min |y|) representative per XZ cell.
std::vector<CellOut>
computeClosestGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud_bl, const GridReq& R);

} // namespace futuraps
