//
// Created by yuta on 2023/09/02.
//
/*
     * The unit of length is [mm], and the unit of angle is [rad].
     * Field origin is set at the front-left corner on the top surface of the field
     * Robot origin is set at the point where the orientation axis of th0 and th1 cross(675, -130, 228)
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
#include <array>
#include <cmath>
#include <iostream>

robot_kinematics::robot_kinematics(){
    link_len[0] = -40;
    link_len[1] =  90;
    link_len[2] = 480;
    link_len[3] = 480;
    link_len[4] =  40;

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
    float _posrot[6];
    convert_field2robot(f_posrot, _posrot);
    using namespace std;

    // Solve the two-link wrist position.  Both 40 mm offsets are horizontal
    // in the arm plane: the base offset is inward and the flange is outward.
    float rxy =
        sqrt(pow(_posrot[X],2) + pow(_posrot[Y],2)) -
        link_len[0] - link_len[4];
    float _z  = _posrot[Z] - link_len[1];
    float l   = sqrt(pow(rxy,2) + pow(_z,2));

    catchrobo_kinematics::JointAngles relative_joint_angles = {{}};
    relative_joint_angles[0] = atan2(_posrot[X], _posrot[Y]);

    const float cosine_elbow =
        (pow(l, 2) - pow(link_len[2], 2) - pow(link_len[3], 2)) /
        (2 * link_len[2] * link_len[3]);

    relative_joint_angles[2] = acos(cosine_elbow);
    relative_joint_angles[1] =
        atan2(rxy, _z) - atan2(
            link_len[3] * sin(relative_joint_angles[2]),
            link_len[2] +
                link_len[3] * cos(relative_joint_angles[2]));
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
