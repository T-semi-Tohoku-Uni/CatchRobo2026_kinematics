#include <memory>

#include "catchrobo2026_msgs/srv/inverse_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_inverse_kinematics/robot_kinematics.h"

namespace {

using InverseKinematics = catchrobo2026_msgs::srv::InverseKinematics;

void calculate_inverse_kinematics(
    const InverseKinematics::Request::SharedPtr request,
    InverseKinematics::Response::SharedPtr response)
{
    robot_kinematics kinematics;
    auto target_pose = request->target_pose;
    kinematics.inverse_kinematics(
        target_pose.data(), response->joint_angles.data());
}

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const auto node = rclcpp::Node::make_shared("inverse_kinematics_server");
    const auto service = node->create_service<InverseKinematics>(
        "inverse_kinematics", calculate_inverse_kinematics);
    (void)service;

    RCLCPP_INFO(node->get_logger(), "Inverse kinematics service is ready");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
