//
// Created by yuta on 2023/09/02.
//
/*
     * The unit of length is [mm], and the unit of angle is [rad].
     * Field origin is set at the front-left corner on the top surface of the field
     * Robot origin is set at the point where the orientation axes of th0 and th1 cross (675, -190, 0).
     * The origin of hand coordinate is set at the point that hand rotation axis and the bottom surface of the endfactor_adapter cross.
     * Positive of field Y axle is set toward opponent.
     * Positive of field X axle is set toward right facing opponent.
     * Joints and links are numbered from the base (0,1,2)
     * Origin of Joint0 is set toward positive field Y axle
     * Origins of Joint1 and 2 are set toward positive field Z axle
     *
     *  o_____________y
     *  |
     *  |
     *  |
     * R|
     *  |
     *  |
     *  |
     *  |_____________
     *  x
     *
     *
     *       /\
     *      /  \
     *     /    \ l1
     * l0 /      \
     *   /        \  l2
     *  /          \_____
     *                |
     *                |l3
     *
     */
//
// Created by yuta on 2023/09/02.
//

#include "ros2_inverse_kinematics/robot_kinematics.h"
#include "ros2_inverse_kinematics/homogeneous_transform.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>

robot_kinematics::robot_kinematics(){
    link_len[0] = catchrobo_kinematics::kBaseRadialOffsetMillimetres;
    link_len[1] = catchrobo_kinematics::kBaseHeightMillimetres;
    link_len[2] = catchrobo_kinematics::kUpperArmLengthMillimetres;
    link_len[3] = catchrobo_kinematics::kForearmLengthMillimetres;
    link_len[4] = catchrobo_kinematics::kFlangeOffsetMillimetres;

    //lower limit                   upper limit
    joint_angle_lim[0][0]=0;    joint_angle_lim[0][1]=2*PI;
    joint_angle_lim[1][0]=0;    joint_angle_lim[1][1]=2*PI;
    joint_angle_lim[2][0]=0;    joint_angle_lim[2][1]=2*PI;
    joint_angle_lim[3][0]=0;    joint_angle_lim[3][1]=2*PI;
}

void robot_kinematics::convert_field2robot(float *f_posrot, float *r_posrot) {
    for(int i=0; i<6; i++){
        r_posrot[i] = f_posrot[i] - robot_pos[i];
    }
}

void robot_kinematics::forward_kinematics(float *posrot, float *joint_angle) {
    const catchrobo_kinematics::JointAngles absolute_joint_angles = {{
        joint_angle[0], joint_angle[1], joint_angle[2], joint_angle[3]
    }};
    const catchrobo_kinematics::JointAngles relative_joint_angles =
        catchrobo_kinematics::absolute_to_relative_joint_angles(
            absolute_joint_angles);
    const catchrobo_kinematics::TransformChain transforms =
        catchrobo_kinematics::make_transform_chain(relative_joint_angles);

    const catchrobo_kinematics::TransformMatrix &field_to_flange =
        transforms[5];

    posrot[X] = static_cast<float>(field_to_flange[3]);
    posrot[Y] = static_cast<float>(field_to_flange[7]);
    posrot[Z] = static_cast<float>(field_to_flange[11]);

    posrot[PHI] = joint_angle[0] + joint_angle[3];
    posrot[THE] = -PI / 2.0F;
    posrot[PSI] = 0;
}

void robot_kinematics::inverse_kinematics(float *f_posrot, float *joint_angle) {
    for (int index = 0; index < 6; ++index) {
        if (!std::isfinite(f_posrot[index])) {
            const float invalid = std::numeric_limits<float>::quiet_NaN();
            std::fill(joint_angle, joint_angle + 4, invalid);
            return;
        }
    }

    float _posrot[6];
    convert_field2robot(f_posrot, _posrot);
    using namespace std;

    // Solve the two-link wrist position after removing fixed offsets.
    const double robot_x = static_cast<double>(_posrot[X]);
    const double robot_y = static_cast<double>(_posrot[Y]);
    const double rxy = std::hypot(robot_x, robot_y) -
        catchrobo_kinematics::kBaseRadialOffsetMillimetres -
        catchrobo_kinematics::kFlangeOffsetMillimetres;
    const double robot_z = static_cast<double>(_posrot[Z]) -
        catchrobo_kinematics::kBaseHeightMillimetres +
        catchrobo_kinematics::kToolVerticalOffsetMillimetres;
    const double wrist_distance = std::hypot(rxy, robot_z);

    constexpr double kFoldedDistanceToleranceMillimetres = 1.0e-6;
    if (wrist_distance <= kFoldedDistanceToleranceMillimetres) {
        const float invalid = std::numeric_limits<float>::quiet_NaN();
        std::fill(joint_angle, joint_angle + 4, invalid);
        return;
    }

    catchrobo_kinematics::JointAngles relative_joint_angles = {{}};
    relative_joint_angles[0] = catchrobo_kinematics::base_angle(
        robot_x, robot_y);

    const double upper_arm =
        catchrobo_kinematics::kUpperArmLengthMillimetres;
    const double forearm =
        catchrobo_kinematics::kForearmLengthMillimetres;
    const double cosine_elbow_unclamped =
        (wrist_distance * wrist_distance - upper_arm * upper_arm -
            forearm * forearm) /
        (2.0 * upper_arm * forearm);

    constexpr double kReachabilityTolerance = 1.0e-5;
    if (!std::isfinite(cosine_elbow_unclamped) ||
        cosine_elbow_unclamped < -1.0 - kReachabilityTolerance ||
        cosine_elbow_unclamped > 1.0 + kReachabilityTolerance) {
        const float invalid = std::numeric_limits<float>::quiet_NaN();
        std::fill(joint_angle, joint_angle + 4, invalid);
        return;
    }
    const double cosine_elbow = std::max(
        -1.0, std::min(1.0, cosine_elbow_unclamped));

    relative_joint_angles[2] = std::acos(cosine_elbow);
    relative_joint_angles[1] =
        std::atan2(rxy, robot_z) - std::atan2(
            forearm * std::sin(relative_joint_angles[2]),
            upper_arm + forearm * std::cos(relative_joint_angles[2]));
    relative_joint_angles[3] = _posrot[PHI] - relative_joint_angles[0];

    const catchrobo_kinematics::JointAngles absolute_joint_angles =
        catchrobo_kinematics::relative_to_absolute_joint_angles(
            relative_joint_angles);
    for (std::size_t index = 0; index < absolute_joint_angles.size(); ++index) {
        joint_angle[index] = static_cast<float>(absolute_joint_angles[index]);
    }
}

void robot_kinematics::get_joint_positions(float *joint_angle, float positions[6][3]) {
    const catchrobo_kinematics::JointAngles absolute_joint_angles = {{
        joint_angle[0], joint_angle[1], joint_angle[2], joint_angle[3]
    }};
    const catchrobo_kinematics::JointAngles relative_joint_angles =
        catchrobo_kinematics::absolute_to_relative_joint_angles(
            absolute_joint_angles);
    
    // Homogeneous transformチェーンを利用して全リンクのフィールド座標を一括で取得
    const catchrobo_kinematics::TransformChain transforms =
        catchrobo_kinematics::make_transform_chain(relative_joint_angles);

    for (int i = 0; i < 6; ++i) {
        positions[i][X] = static_cast<float>(transforms[i][3]);
        positions[i][Y] = static_cast<float>(transforms[i][7]);
        positions[i][Z] = static_cast<float>(transforms[i][11]);
    }
}
