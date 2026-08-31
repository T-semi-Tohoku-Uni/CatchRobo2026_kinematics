#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "ros2_inverse_kinematics/homogeneous_transform.h"
#include "ros2_inverse_kinematics/robot_kinematics.h"
#include "ros2_inverse_kinematics/target_pose_csv.h"

namespace {

struct FileSpecification {
    const char *filename;
    catchrobo_kinematics::TargetPoseKind kind;
    catchrobo_kinematics::TeamSide team_side;
};

struct MockPose {
    struct Position {
        double x;
        double y;
        double z;
    } position;
    struct Orientation {
        double x;
        double y;
        double z;
        double w;
    } orientation;
};

struct MockInverseKinematicsRequest {
    std::array<float, 6> target_pose;
};

float wrapped_difference(float left, float right)
{
    return std::atan2(std::sin(left - right), std::cos(left - right));
}

bool joint_angles_are_finite(const float *joint_angles)
{
    for (int index = 0; index < 4; ++index) {
        if (!std::isfinite(joint_angles[index])) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Usage: test_target_pose_csv <flange-target-directory>\n";
        return 2;
    }

    const std::array<FileSpecification, 4> specifications = {{
        {"pick_pose_red.csv", catchrobo_kinematics::TargetPoseKind::Pick,
            catchrobo_kinematics::TeamSide::Red},
        {"pick_pose_blue.csv", catchrobo_kinematics::TargetPoseKind::Pick,
            catchrobo_kinematics::TeamSide::Blue},
        {"shooting_pose_red.csv",
            catchrobo_kinematics::TargetPoseKind::Shooting,
            catchrobo_kinematics::TeamSide::Red},
        {"shooting_pose_blue.csv",
            catchrobo_kinematics::TargetPoseKind::Shooting,
            catchrobo_kinematics::TeamSide::Blue}
    }};

    robot_kinematics kinematics;
    bool passed = true;

    for (std::size_t file_index = 0;
         file_index < specifications.size(); ++file_index) {
        const FileSpecification &specification = specifications[file_index];
        const std::string path =
            std::string(argv[1]) + "/" + specification.filename;
        const std::vector<catchrobo_kinematics::TargetPose> poses =
            catchrobo_kinematics::load_target_pose_csv(
                path, specification.kind, specification.team_side);

        std::size_t reachable_count = 0;
        double maximum_position_error = 0.0;
        double maximum_theta_error = 0.0;
        double maximum_orientation_matrix_error = 0.0;

        for (std::size_t pose_index = 0;
             pose_index < poses.size(); ++pose_index) {
            const catchrobo_kinematics::TargetPose &target = poses[pose_index];
            std::array<float, 6> ik_pose = target.inverse_kinematics_pose();
            float joint_angles[4] = {};
            kinematics.inverse_kinematics(ik_pose.data(), joint_angles);
            if (!joint_angles_are_finite(joint_angles)) {
                continue;
            }
            ++reachable_count;

            float reconstructed_pose[6] = {};
            kinematics.forward_kinematics(
                reconstructed_pose, joint_angles);
            const double dx = reconstructed_pose[X] - target.x_millimetres;
            const double dy = reconstructed_pose[Y] - target.y_millimetres;
            const double dz = reconstructed_pose[Z] - target.z_millimetres;
            maximum_position_error = std::max(
                maximum_position_error, std::sqrt(dx * dx + dy * dy + dz * dz));
            maximum_theta_error = std::max(
                maximum_theta_error,
                static_cast<double>(std::fabs(wrapped_difference(
                    reconstructed_pose[PHI],
                    static_cast<float>(target.theta_radians)))));

            const std::array<double, 4> independent_joint_angles = {{
                joint_angles[0], joint_angles[1],
                joint_angles[2], joint_angles[3]
            }};
            const catchrobo_kinematics::TransformChain transforms =
                catchrobo_kinematics::make_transform_chain(
                    independent_joint_angles);
            const catchrobo_kinematics::TransformMatrix expected_orientation =
                catchrobo_kinematics::rotation_z(target.theta_radians);
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    maximum_orientation_matrix_error = std::max(
                        maximum_orientation_matrix_error,
                        std::fabs(
                            transforms[5][row * 4 + column] -
                            expected_orientation[row * 4 + column]));
                }
            }

            const std::array<double, 3> metres = target.position_metres();
            const std::array<double, 4> quaternion =
                target.yaw_quaternion_xyzw();
            MockPose ros_pose{};
            target.assign_ros_pose(ros_pose);
            MockInverseKinematicsRequest request{};
            target.assign_inverse_kinematics_request(request);
            const double quaternion_norm = std::sqrt(
                quaternion[0] * quaternion[0] +
                quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] +
                quaternion[3] * quaternion[3]);
            if (std::fabs(metres[0] * 1000.0 - target.x_millimetres) > 1.0e-9 ||
                std::fabs(metres[1] * 1000.0 - target.y_millimetres) > 1.0e-9 ||
                std::fabs(metres[2] * 1000.0 - target.z_millimetres) > 1.0e-9 ||
                std::fabs(quaternion_norm - 1.0) > 1.0e-12 ||
                std::fabs(ros_pose.position.x - metres[0]) > 1.0e-12 ||
                std::fabs(ros_pose.position.y - metres[1]) > 1.0e-12 ||
                std::fabs(ros_pose.position.z - metres[2]) > 1.0e-12 ||
                std::fabs(ros_pose.orientation.z - quaternion[2]) > 1.0e-12 ||
                std::fabs(ros_pose.orientation.w - quaternion[3]) > 1.0e-12 ||
                request.target_pose != ik_pose) {
                passed = false;
            }
        }

        if (reachable_count != poses.size() ||
            maximum_position_error > 1.0e-2 ||
            maximum_theta_error > 1.0e-5 ||
            maximum_orientation_matrix_error > 1.0e-6) {
            passed = false;
        }

        std::cout << specification.filename
                  << ": rows=" << poses.size()
                  << ", reachable=" << reachable_count
                  << ", max_position_error_mm=" << std::fixed
                  << std::setprecision(6) << maximum_position_error
                  << ", max_theta_error_rad=" << maximum_theta_error
                  << ", max_orientation_matrix_error="
                  << maximum_orientation_matrix_error << '\n';

        const std::array<int, 3> sample_ids = {{1, 12, 24}};
        for (std::size_t sample_index = 0;
             sample_index < sample_ids.size(); ++sample_index) {
            const catchrobo_kinematics::TargetPose &sample =
                catchrobo_kinematics::find_target_by_id(
                    poses, sample_ids[sample_index]);
            std::array<float, 6> ik_pose = sample.inverse_kinematics_pose();
            float joint_angles[4] = {};
            kinematics.inverse_kinematics(ik_pose.data(), joint_angles);
            std::cout << "  ID=" << sample.id
                      << " target_mm=(" << sample.x_millimetres << ", "
                      << sample.y_millimetres << ", "
                      << sample.z_millimetres << ")"
                      << " joints_rad=(" << joint_angles[0] << ", "
                      << joint_angles[1] << ", " << joint_angles[2] << ", "
                      << joint_angles[3] << ")\n";
        }
    }

    if (!passed) {
        std::cerr << "Target pose CSV validation failed\n";
        return 1;
    }

    std::cout << "All target pose CSV and IK/FK checks passed\n";
    return 0;
}
