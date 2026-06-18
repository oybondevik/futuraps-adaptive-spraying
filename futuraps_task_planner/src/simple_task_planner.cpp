#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

class SimpleTaskPlanner : public rclcpp::Node
{
public:
    SimpleTaskPlanner() 
    : Node("simple_task_planner")
    {
        
        timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {on_timer(); });
    }

private:

    void on_timer()
    {
        if(!move_group_)
        {
            using moveit::planning_interface::MoveGroupInterface;

            move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "ur10_arm");

            RCLCPP_INFO(get_logger(), "SimpleTaskPlanner initialized");
        }

        send_goal();
        timer_->cancel();

    }

    void send_goal()
    {
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose.position.x = 0.5;
        target.pose.position.y = 0.0;
        target.pose.position.z = 0.4;
        target.pose.orientation.w = 1.0;

        move_group_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_->plan(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Planning succeeded, executing");
            move_group_->execute(plan);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Planning failed");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
