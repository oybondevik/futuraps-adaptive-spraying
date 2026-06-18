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

#pragma once

namespace futuraps_spray_coverage
{

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct SprayModelConfig
{
  double peak_intensity{1.0};
  double fan_angle_deg{80.0};
  double narrow_angle_deg{35.0};
  double max_range{1.0};
  bool use_distance_attenuation{true};
  double reference_distance{0.5};
  double min_distance{0.1};
  double attenuation_exponent{2.0};
  double max_attenuation{10.0};
  double min_attenuation{0.01};
};

struct SprayFrame
{
  Vector3 origin;
  Vector3 direction;
  Vector3 wide;
  Vector3 narrow;
};

struct SprayProjection
{
  double s{0.0};
  double p_narrow{0.0};
  double p_wide{0.0};
  double ellipse_r2{0.0};
  bool inside_footprint{false};
};

double wideSemiAxisAtDistance(double s, const SprayModelConfig & config);

double narrowSemiAxisAtDistance(double s, const SprayModelConfig & config);

SprayProjection projectPointToSpray(
  const Vector3 & point,
  const SprayFrame & frame,
  const SprayModelConfig & config);

double computeIntensity(
  const SprayProjection & projection,
  const SprayModelConfig & config);

}  // namespace futuraps_spray_coverage
