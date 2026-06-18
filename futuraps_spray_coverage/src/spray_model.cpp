// Copyright 2026 Oystein Bondevik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "futuraps_spray_coverage/spray_model.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double kSemiAxisEps = 1e-6;
constexpr double kPi = 3.14159265358979323846;

double degToRad(double deg)
{
  return deg * kPi / 180.0;
}

double dot(
  const futuraps_spray_coverage::Vector3 & a,
  const futuraps_spray_coverage::Vector3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

}  // namespace

namespace futuraps_spray_coverage
{

double wideSemiAxisAtDistance(double s, const SprayModelConfig & config)
{
  return s * std::tan(degToRad(config.fan_angle_deg / 2.0));
}

double narrowSemiAxisAtDistance(double s, const SprayModelConfig & config)
{
  return s * std::tan(degToRad(config.narrow_angle_deg / 2.0));
}

SprayProjection projectPointToSpray(
  const Vector3 & point,
  const SprayFrame & frame,
  const SprayModelConfig & config)
{
  const Vector3 r{
    point.x - frame.origin.x,
    point.y - frame.origin.y,
    point.z - frame.origin.z};

  SprayProjection projection;
  projection.s = dot(r, frame.direction);

  if (projection.s <= 0.0 || projection.s > config.max_range) {
    return projection;
  }

  const double wide_semi_axis =
    wideSemiAxisAtDistance(projection.s, config);
  const double narrow_semi_axis =
    narrowSemiAxisAtDistance(projection.s, config);

  if (wide_semi_axis <= kSemiAxisEps || narrow_semi_axis <= kSemiAxisEps) {
    return projection;
  }

  projection.p_narrow = dot(r, frame.narrow);
  projection.p_wide = dot(r, frame.wide);

  projection.ellipse_r2 =
    (projection.p_wide * projection.p_wide) /
    (wide_semi_axis * wide_semi_axis) +
    (projection.p_narrow * projection.p_narrow) /
    (narrow_semi_axis * narrow_semi_axis);
  projection.inside_footprint = projection.ellipse_r2 <= 1.0;

  return projection;
}

double computeIntensity(
  const SprayProjection & projection,
  const SprayModelConfig & config)
{
  if (!projection.inside_footprint) {
    return 0.0;
  }

  const double base_intensity =
    config.peak_intensity * (1.0 - projection.ellipse_r2);

  if (!config.use_distance_attenuation) {
    return std::max(0.0, base_intensity);
  }

  const double distance = std::max(projection.s, config.min_distance);
  const double unclamped_attenuation =
    std::pow(config.reference_distance / distance, config.attenuation_exponent);
  const double attenuation = std::clamp(
    unclamped_attenuation,
    config.min_attenuation,
    config.max_attenuation);

  return std::max(0.0, base_intensity * attenuation);
}

}  // namespace futuraps_spray_coverage
