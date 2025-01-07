#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class RobotMovementNode : public rclcpp::Node
{
public:
    RobotMovementNode() : Node("robot_movement")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        
        send_trajectory();
    }

    void send_trajectory()
    {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};  // Example joint positions
        point.time_from_start.sec = 1;
        msg.points.push_back(point);

        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMovementNode>());
    rclcpp::shutdown();
    return 0;
}

