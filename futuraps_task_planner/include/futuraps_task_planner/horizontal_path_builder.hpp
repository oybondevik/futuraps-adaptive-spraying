#pragma once

#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "futuraps_task_planner/types.hpp"

namespace futuraps_task_planner
{

class HorizontalPathBuilder{
public:
    std::vector<geometry_msgs::msg::Pose> buildWaypoints(
        const geometry_msgs::msg::Pose& current_pose,
        const PerceptionResult& perception,
        const HorizontalPathConfig& config) const;
 
private:
    static bool finite3(double a, double b, double c);

    static tf2::Quaternion quatAlignToolAxisToForward(
        const tf2::Vector3& forward_in,
        const tf2::Vector3& up_hint_in,
        int tool_axis);
};


} // namespace futuraps_task_planner
