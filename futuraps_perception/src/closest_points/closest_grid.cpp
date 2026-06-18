#include "futuraps_perception/closest_grid.hpp"
#include <algorithm>
#include <cmath>

namespace futuraps {

static inline std::pair<int,int> bin_idx(float x, float z, const GridReq& R){
  int j = int(std::floor((x - R.x0)/R.s)); // cols along +X
  int i = int(std::floor((z - R.z0)/R.s)); // rows along +Z
  return {i,j};
}

std::vector<CellOut>
computeClosestGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud, const GridReq& R)
{
  std::vector<std::vector<const pcl::PointXYZ*>> bins(R.rows * R.cols);

  // 1) Bin points (filter laterally and by side)
  for (const auto& p : cloud.points) {
    if (p.y < -R.yR || p.y > R.yL) continue;   // lateral limits
    if (R.side == 1  && p.y < 0) continue;     // left-only
    if (R.side == -1 && p.y > 0) continue;     // right-only
    auto [i,j] = bin_idx(p.x, p.z, R);
    if (i<0 || i>=int(R.rows) || j<0 || j>=int(R.cols)) continue;
    bins[i*R.cols + j].push_back(&p);
  }

  // 2) Pick representative near the front percentile of |y|
  std::vector<CellOut> out(R.rows * R.cols);
  for (size_t k=0; k<bins.size(); ++k) {
    auto& cell = bins[k];
    if (cell.size() < R.min_pts) { out[k].found=false; continue; }

    std::vector<float> ay; ay.reserve(cell.size());
    for (auto* P : cell) ay.push_back(std::abs(P->y));
    size_t kk = std::min(ay.size()-1, size_t(std::floor(R.p_front * ay.size())));
    std::nth_element(ay.begin(), ay.begin()+kk, ay.end());
    float thr = ay[kk];

    const pcl::PointXYZ* best=nullptr;
    float best_err = 1e9f;
    for (auto* P : cell) {
      float err = std::abs(std::abs(P->y) - thr);
      if (err < best_err) { best = P; best_err = err; }
    }

    if (best) {
      out[k].found  = true;
      out[k].x      = best->x;
      out[k].y      = best->y;     // signed (>=0 left, <=0 right)
      out[k].z      = best->z;
      out[k].dist   = std::abs(best->y);
      out[k].count  = static_cast<uint32_t>(cell.size());
      out[k].confidence = std::min(1.f, float(cell.size())/float(std::max<uint32_t>(1, R.min_pts)));
    }
  }
  return out;
}

} // namespace futuraps
