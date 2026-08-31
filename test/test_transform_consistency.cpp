#include <array>
#include <cmath>
#include <iostream>

#include "ros2_inverse_kinematics/homogeneous_transform.h"
#include "ros2_inverse_kinematics/robot_kinematics.h"

namespace {

float wrapped_difference(float left, float right)
{
    return std::atan2(std::sin(left - right), std::cos(left - right));
}

bool nearly_equal(float left, float right, float tolerance)
{
    return std::fabs(left - right) <= tolerance;
}

}  // namespace

int main()
{
    const std::array<std::array<float, 4>, 4> cases = {{
        {{0.0F, 0.0F, 0.0F, 0.0F}},
        {{0.4F, -0.2F, 0.8F, 0.3F}},
        {{-0.7F, 0.15F, 1.1F, -0.5F}},
        {{1.2F, -0.4F, 1.5F, 0.6F}}
    }};

    robot_kinematics kinematics;
    bool passed = true;

    for (std::size_t index = 0; index < cases.size(); ++index) {
        float input_joint_angles[4] = {
            cases[index][0], cases[index][1],
            cases[index][2], cases[index][3]
        };
        float flange_pose[6] = {};
        float solved_joint_angles[4] = {};

        kinematics.forward_kinematics(flange_pose, input_joint_angles);
        kinematics.inverse_kinematics(flange_pose, solved_joint_angles);

        const std::array<double, 4> independent = {{
            input_joint_angles[0], input_joint_angles[1],
            input_joint_angles[2], input_joint_angles[3]
        }};
        const catchrobo_kinematics::TransformChain transforms =
            catchrobo_kinematics::make_transform_chain(independent);

        const bool matrix_matches_pose =
            nearly_equal(flange_pose[X], transforms[5][3], 1.0e-3F) &&
            nearly_equal(flange_pose[Y], transforms[5][7], 1.0e-3F) &&
            nearly_equal(flange_pose[Z], transforms[5][11], 1.0e-3F);

        const catchrobo_kinematics::TransformMatrix expected_orientation =
            catchrobo_kinematics::rotation_z(
                input_joint_angles[0] + input_joint_angles[3]);
        bool orientation_matches_task_yaw = true;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                orientation_matches_task_yaw =
                    orientation_matches_task_yaw && nearly_equal(
                        transforms[5][row * 4 + column],
                        expected_orientation[row * 4 + column], 1.0e-6F);
            }
        }

        const bool ik_matches_input =
            std::fabs(wrapped_difference(
                solved_joint_angles[0], input_joint_angles[0])) < 1.0e-4F &&
            std::fabs(wrapped_difference(
                solved_joint_angles[1], input_joint_angles[1])) < 1.0e-4F &&
            std::fabs(wrapped_difference(
                solved_joint_angles[2], input_joint_angles[2])) < 1.0e-4F &&
            std::fabs(wrapped_difference(
                solved_joint_angles[3], input_joint_angles[3])) < 1.0e-4F;

        const double theta2_prime =
            catchrobo_kinematics::dependent_theta2_prime(
                input_joint_angles[1], input_joint_angles[2]);
        const bool constraint_holds = std::fabs(
            theta2_prime + input_joint_angles[1] +
            input_joint_angles[2] + catchrobo_kinematics::kPi / 2.0) <
            1.0e-12;

        if (!matrix_matches_pose || !orientation_matches_task_yaw ||
            !ik_matches_input || !constraint_holds) {
            std::cerr << "Consistency test failed for case " << index << '\n';
            passed = false;
        }
    }

    if (!passed) {
        return 1;
    }

    std::cout << "All FK/IK transform consistency tests passed\n";
    return 0;
}
