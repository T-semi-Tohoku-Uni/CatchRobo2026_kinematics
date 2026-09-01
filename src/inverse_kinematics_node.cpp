#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <memory>
#include "ros2_inverse_kinematics/robot_kinematics.h"
#include "catchrobo2026_msgs/srv/inverse_kinematics.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std;
using IKSrv = catchrobo2026_msgs::srv::InverseKinematics;

class InverseKinematicsNode : public rclcpp::Node {
public:
  InverseKinematicsNode() : Node("inverse_kinematics_server") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ik_markers", 10);
    
    service_ = this->create_service<IKSrv>(
      "inverse_kinematics",
      std::bind(&InverseKinematicsNode::calc_inverse_kinematics, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Inverse kinematics ready");
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<IKSrv>::SharedPtr service_;
  robot_kinematics robot_kin;

  void calc_inverse_kinematics(const std::shared_ptr<IKSrv::Request> request,
                               std::shared_ptr<IKSrv::Response> response) {
    float target_pos[6] = {
        request->target_pose[0], request->target_pose[1], request->target_pose[2],
        request->target_pose[3], request->target_pose[4], request->target_pose[5]
    };
    float joint_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    robot_kin.inverse_kinematics(target_pos, joint_angle);

    response->joint_angles[0] = joint_angle[0];
    response->joint_angles[1] = joint_angle[1];
    response->joint_angles[2] = joint_angle[2];
    response->joint_angles[3] = joint_angle[3];

    publish_markers(joint_angle, target_pos);
  }

  void publish_markers(float *joint_angle, float *target_pos) {
    float positions[6][3];
    robot_kin.get_joint_positions(joint_angle, positions);

    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time now = this->now();

    // frame 0 and frame 1 share the theta1 rotation point.  Delete the old
    // zero-length arrow instead of rendering it as a misleading red dot.
    visualization_msgs::msg::Marker obsolete_base_marker;
    obsolete_base_marker.header.frame_id = "map";
    obsolete_base_marker.header.stamp = now;
    obsolete_base_marker.ns = "ik_links";
    obsolete_base_marker.id = 0;
    obsolete_base_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(obsolete_base_marker);

    // 1. 各リンクの描画
    // TransformChainから得た座標は既にフィールド座標（robot_posオフセット加算済み）になっています
    for (int i = 1; i < 5; ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = now;
      marker.ns = "ik_links";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = positions[i][0] / 1000.0;
      p_start.y = positions[i][1] / 1000.0;
      p_start.z = positions[i][2] / 1000.0;

      p_end.x = positions[i+1][0] / 1000.0;
      p_end.y = positions[i+1][1] / 1000.0;
      p_end.z = positions[i+1][2] / 1000.0;

      marker.points.push_back(p_start);
      marker.points.push_back(p_end);

      marker.scale.x = 0.02; 
      marker.scale.y = 0.04; 
      marker.scale.z = 0.04; 
      marker.color.a = 1.0;
      marker.color.r = (i == 0) ? 1.0 : 0.0;
      marker.color.g = (i == 1) ? 1.0 : 0.0;
      marker.color.b = (i >= 2) ? 1.0 : 0.0;

      marker_array.markers.push_back(marker);
    }

    // 2. 目標位置の描画
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = "map";
    target_marker.header.stamp = now;
    target_marker.ns = "ik_target"; 
    target_marker.id = 5;           
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;

    target_marker.pose.position.x = target_pos[0] / 1000.0;
    target_marker.pose.position.y = target_pos[1] / 1000.0;
    target_marker.pose.position.z = target_pos[2] / 1000.0;
    target_marker.pose.orientation.w = 1.0;

    target_marker.scale.x = 0.05;
    target_marker.scale.y = 0.05;
    target_marker.scale.z = 0.05;
    target_marker.color.a = 0.8; 
    target_marker.color.r = 1.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 1.0;

    marker_array.markers.push_back(target_marker);

    marker_pub_->publish(marker_array);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
