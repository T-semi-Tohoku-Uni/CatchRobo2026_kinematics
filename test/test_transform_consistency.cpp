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
        {{-1.0, 0.0, -catchrobo_kinematics::kPi / 2.0}},
        {{0.0, -1.0, catchrobo_kinematics::kPi}},
        {{1.0, 0.0, catchrobo_kinematics::kPi / 2.0}}
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
        {{0.0F, 0.0F, 0.0F, 675.0F, -190.0F, 1050.0F, 0.0F}},
        {{0.0F, static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            675.0F, 770.0F, 90.0F, 0.0F}},
        {{static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            static_cast<float>(catchrobo_kinematics::kPi / 2.0),
            1635.0F, -190.0F, 90.0F,
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

    struct LegacyIkCase {
        std::array<float, 6> pose;
        std::array<float, 4> expected_joints;
        float tolerance;
    };
    constexpr float degrees_to_radians =
        static_cast<float>(catchrobo_kinematics::kPi / 180.0);
    const std::array<LegacyIkCase, 4> legacy_ik_cases = {{
        {{{670.0F, -110.0F, 220.0F, 0.0F, 0.0F, 0.0F}},
            {{-3.576334F * degrees_to_radians,
              -49.188615F * degrees_to_radians,
              112.503365F * degrees_to_radians,
              3.576334F * degrees_to_radians}}, 2.0e-5F},
        {{{670.0F, -110.0F, 220.0F,
            -2.0F * static_cast<float>(catchrobo_kinematics::kPi), 0.0F, 0.0F}},
            {{-3.576334F * degrees_to_radians,
              -49.188615F * degrees_to_radians,
              112.503365F * degrees_to_radians,
              -356.423668F * degrees_to_radians}}, 2.0e-5F},
        {{{175.0F, 138.0F, 166.95F,
            -static_cast<float>(catchrobo_kinematics::kPi / 2.0), 0.0F, 0.0F}},
            {{-0.990214705467224F, 0.551046848297119F,
              2.33458733558655F, -0.580581665039062F}}, 1.0e-5F},
        {{{1256.2F, -71.65F, 294.35F,
            -static_cast<float>(catchrobo_kinematics::kPi / 2.0), 0.0F, 0.0F}},
            {{1.36991238594055F, 0.380382359027863F,
              2.09762382507324F, -2.94070863723755F}}, 1.0e-5F}
    }};
    for (std::size_t index = 0; index < legacy_ik_cases.size(); ++index) {
        float joints[4] = {};
        std::array<float, 6> pose = legacy_ik_cases[index].pose;
        kinematics.inverse_kinematics(pose.data(), joints);
        bool matches = all_finite(joints, 4);
        for (std::size_t joint = 0; joint < 4; ++joint) {
            const float expected = legacy_ik_cases[index].expected_joints[joint];
            matches = matches &&
                std::fabs(joints[joint] - expected) <= legacy_ik_cases[index].tolerance;
        }
        if (!matches) {
            std::cerr << "Legacy non-wrapped IK output changed for case " << index << '\n';
            for (std::size_t joint = 0; joint < 4; ++joint) {
                std::cerr << "  joint " << joint << ": " << joints[joint]
                          << " expected "
                          << legacy_ik_cases[index].expected_joints[joint]
                          << '\n';
            }
            passed = false;
        }
    }

    float near_fold_pose[6] = {675.0F, -190.0F, 90.1F, 0.0F, 0.0F, 0.0F};
    float near_fold_joints[4] = {};
    float near_fold_reconstructed[6] = {};
    kinematics.inverse_kinematics(near_fold_pose, near_fold_joints);
    kinematics.forward_kinematics(near_fold_reconstructed, near_fold_joints);
    if (!all_finite(near_fold_joints, 4) ||
        !nearly_equal(near_fold_reconstructed[X], near_fold_pose[X], 1.0e-3F) ||
        !nearly_equal(near_fold_reconstructed[Y], near_fold_pose[Y], 1.0e-3F) ||
        !nearly_equal(near_fold_reconstructed[Z], near_fold_pose[Z], 1.0e-3F)) {
        std::cerr << "Near-fold IK lost double-precision reconstruction\n";
        passed = false;
    }

    float folded_pose[6] = {675.0F, -190.0F, 90.0F, 0.0F, 0.0F, 0.0F};
    float unreachable_pose[6] = {5000.0F, -190.0F, 90.0F, 0.0F, 0.0F, 0.0F};
    float invalid_pose[6] = {675.0F, -190.0F, 91.0F, 0.0F, 0.0F, 0.0F};
    invalid_pose[X] = std::numeric_limits<float>::infinity();
    float folded_joints[4] = {};
    float unreachable_joints[4] = {};
    float invalid_joints[4] = {};
    kinematics.inverse_kinematics(folded_pose, folded_joints);
    kinematics.inverse_kinematics(unreachable_pose, unreachable_joints);
    kinematics.inverse_kinematics(invalid_pose, invalid_joints);
    if (!all_nan(folded_joints, 4) || !all_nan(unreachable_joints, 4) ||
        !all_nan(invalid_joints, 4)) {
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
