#include <array>
#include <cmath>
#include <iostream>
#include <limits>

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

bool all_finite(const float *values, std::size_t count)
{
    for (std::size_t index = 0; index < count; ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

bool all_nan(const float *values, std::size_t count)
{
    for (std::size_t index = 0; index < count; ++index) {
        if (!std::isnan(values[index])) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main()
{
    const std::array<std::array<float, 4>, 4> cases = {{
        {{0.0F, 0.2F, 0.8F, 0.0F}},
        {{0.4F, -0.2F, 0.8F, 0.3F}},
        {{-0.7F, 0.15F, 1.1F, -0.5F}},
        {{1.2F, -0.4F, 1.5F, 0.6F}}
    }};

    robot_kinematics kinematics;
    bool passed = true;

    const std::array<std::array<double, 3>, 4> base_angle_cases = {{
        {{0.0, 1.0, 0.0}},
        {{-1.0, 0.0, catchrobo_kinematics::kPi / 2.0}},
        {{0.0, -1.0, catchrobo_kinematics::kPi}},
        {{1.0, 0.0, -catchrobo_kinematics::kPi / 2.0}}
    }};
    for (std::size_t index = 0; index < base_angle_cases.size(); ++index) {
        if (std::fabs(wrapped_difference(
                static_cast<float>(catchrobo_kinematics::base_angle(
                    base_angle_cases[index][0], base_angle_cases[index][1])),
                static_cast<float>(base_angle_cases[index][2]))) > 1.0e-6F) {
            std::cerr << "Base-angle convention failed for case " << index << '\n';
            passed = false;
        }
    }

    const std::array<std::array<float, 7>, 3> known_forward_cases = {{
        {{0.0F, 0.0F, 0.0F, 675.0F, -90.0F, 1188.0F, 0.0F}},
        {{0.0F, static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            675.0F, 870.0F, 228.0F, 0.0F}},
        {{static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            -325.0F, -130.0F, 228.0F,
            static_cast<float>(catchrobo_kinematics::kPi / 2.0)}}
    }};
    for (std::size_t index = 0; index < known_forward_cases.size(); ++index) {
        float joints[4] = {
            known_forward_cases[index][0], known_forward_cases[index][1],
            known_forward_cases[index][2], 0.0F
        };
        float pose[6] = {};
        kinematics.forward_kinematics(pose, joints);
        if (!nearly_equal(pose[X], known_forward_cases[index][3], 1.0e-3F) ||
            !nearly_equal(pose[Y], known_forward_cases[index][4], 1.0e-3F) ||
            !nearly_equal(pose[Z], known_forward_cases[index][5], 1.0e-3F) ||
            !nearly_equal(pose[PHI], known_forward_cases[index][6], 1.0e-6F)) {
            std::cerr << "Known FK case failed for case " << index << '\n';
            passed = false;
        }
    }

    float before_crossing[6] = {675.0F, -90.0F, 229.0F, 0.0F, 0.0F, 0.0F};
    float after_crossing[6] = {675.0F, -90.0F, 228.1F, 0.0F, 0.0F, 0.0F};
    float before_joints[4] = {};
    float after_joints[4] = {};
    kinematics.inverse_kinematics(before_crossing, before_joints);
    kinematics.inverse_kinematics(after_crossing, after_joints);
    float reconstructed_before[6] = {};
    float reconstructed_after[6] = {};
    kinematics.forward_kinematics(reconstructed_before, before_joints);
    kinematics.forward_kinematics(reconstructed_after, after_joints);
    if (!all_finite(before_joints, 4) || !all_finite(after_joints, 4) ||
        std::fabs(after_joints[1] - before_joints[1]) > 0.1F ||
        std::fabs(after_joints[2] - before_joints[2]) > 0.1F ||
        !nearly_equal(reconstructed_before[X], before_crossing[X], 1.0e-3F) ||
        !nearly_equal(reconstructed_before[Y], before_crossing[Y], 1.0e-3F) ||
        !nearly_equal(reconstructed_before[Z], before_crossing[Z], 1.0e-3F) ||
        !nearly_equal(reconstructed_after[X], after_crossing[X], 1.0e-3F) ||
        !nearly_equal(reconstructed_after[Y], after_crossing[Y], 1.0e-3F) ||
        !nearly_equal(reconstructed_after[Z], after_crossing[Z], 1.0e-3F)) {
        std::cerr << "Shoulder angles jump across the flange-radius boundary\n";
        passed = false;
    }

    float inside_radius[6] = {675.0F, -91.0F, 220.0F, 0.0F, 0.0F, 0.0F};
    float outside_radius[6] = {675.0F, -89.0F, 220.0F, 0.0F, 0.0F, 0.0F};
    float inside_joints[4] = {};
    float outside_joints[4] = {};
    kinematics.inverse_kinematics(inside_radius, inside_joints);
    kinematics.inverse_kinematics(outside_radius, outside_joints);
    if (!all_finite(inside_joints, 4) || !all_finite(outside_joints, 4) ||
        std::fabs(outside_joints[1] - inside_joints[1]) > 1.0F ||
        std::fabs(outside_joints[2] - inside_joints[2]) > 1.0F) {
        std::cerr << "Shoulder branch jumps across zero radial distance\n";
        passed = false;
    }

    float folded_pose[6] = {675.0F, -90.0F, 228.0F, 0.0F, 0.0F, 0.0F};
    float invalid_pose[6] = {675.0F, -90.0F, 229.0F, 0.0F, 0.0F, 0.0F};
    invalid_pose[X] = std::numeric_limits<float>::infinity();
    float folded_joints[4] = {};
    float invalid_joints[4] = {};
    kinematics.inverse_kinematics(folded_pose, folded_joints);
    kinematics.inverse_kinematics(invalid_pose, invalid_joints);
    if (!all_nan(folded_joints, 4) || !all_nan(invalid_joints, 4)) {
        std::cerr << "Invalid IK input was not rejected\n";
        passed = false;
    }

    {
        const catchrobo_kinematics::JointAngles vertical_relative = {{
            0.0, 0.0, 0.0, 0.0
        }};
        const catchrobo_kinematics::TransformChain vertical =
            catchrobo_kinematics::make_transform_chain(vertical_relative);
        const bool zero_angles_point_up =
            nearly_equal(
                static_cast<float>(vertical[3][11]),
                static_cast<float>(
                    catchrobo_kinematics::kRobotZMillimetres +
                    catchrobo_kinematics::kBaseHeightMillimetres +
                    catchrobo_kinematics::kUpperArmLengthMillimetres),
                1.0e-4F) &&
            nearly_equal(
                static_cast<float>(vertical[4][11]),
                static_cast<float>(
                    catchrobo_kinematics::kRobotZMillimetres +
                    catchrobo_kinematics::kBaseHeightMillimetres +
                    catchrobo_kinematics::kUpperArmLengthMillimetres +
                    catchrobo_kinematics::kForearmLengthMillimetres),
                1.0e-4F) &&
            nearly_equal(
                static_cast<float>(vertical[5][11]),
                static_cast<float>(
                    catchrobo_kinematics::kRobotZMillimetres +
                    catchrobo_kinematics::kBaseHeightMillimetres +
                    catchrobo_kinematics::kUpperArmLengthMillimetres +
                    catchrobo_kinematics::kForearmLengthMillimetres),
                1.0e-4F) &&
            nearly_equal(
                static_cast<float>(vertical[5][3]),
                static_cast<float>(catchrobo_kinematics::kRobotXMillimetres),
                1.0e-4F) &&
            nearly_equal(
                static_cast<float>(vertical[5][7] - vertical[4][7]),
                static_cast<float>(
                    catchrobo_kinematics::kFlangeOffsetMillimetres),
                1.0e-4F);
        if (!zero_angles_point_up) {
            std::cerr << "Zero-angle links do not point along field Z+\n";
            passed = false;
        }
    }

    for (std::size_t index = 0; index < cases.size(); ++index) {
        float input_joint_angles[4] = {
            cases[index][0], cases[index][1],
            cases[index][2], cases[index][3]
        };
        float flange_pose[6] = {};
        float solved_joint_angles[4] = {};

        kinematics.forward_kinematics(flange_pose, input_joint_angles);
        kinematics.inverse_kinematics(flange_pose, solved_joint_angles);

        const catchrobo_kinematics::JointAngles absolute = {{
            input_joint_angles[0], input_joint_angles[1],
            input_joint_angles[2], input_joint_angles[3]
        }};
        const catchrobo_kinematics::JointAngles relative =
            catchrobo_kinematics::absolute_to_relative_joint_angles(absolute);
        const catchrobo_kinematics::JointAngles round_trip_absolute =
            catchrobo_kinematics::relative_to_absolute_joint_angles(relative);
        const bool conversion_matches_definition =
            nearly_equal(
                static_cast<float>(relative[1]),
                input_joint_angles[1], 1.0e-6F) &&
            nearly_equal(
                static_cast<float>(relative[2]),
                input_joint_angles[2] - input_joint_angles[1], 1.0e-6F);
        const catchrobo_kinematics::TransformChain transforms =
            catchrobo_kinematics::make_transform_chain(relative);

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

        bool angle_conversion_round_trip = true;
        for (std::size_t joint = 0; joint < absolute.size(); ++joint) {
            angle_conversion_round_trip = angle_conversion_round_trip &&
                nearly_equal(
                    static_cast<float>(round_trip_absolute[joint]),
                    input_joint_angles[joint], 1.0e-6F);
        }

        const double theta2_prime =
            catchrobo_kinematics::dependent_theta2_prime(
                relative[1], relative[2]);
        const bool constraint_holds = std::fabs(
            theta2_prime + relative[1] + relative[2] +
            catchrobo_kinematics::kPi / 2.0) <
            1.0e-12;

        if (!matrix_matches_pose || !orientation_matches_task_yaw ||
            !ik_matches_input || !conversion_matches_definition ||
            !angle_conversion_round_trip ||
            !constraint_holds) {
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
