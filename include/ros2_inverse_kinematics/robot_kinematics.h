//
// Created by yuta on 2023/09/02.
//

#ifndef CATCHROBO2023_ROBOT_KINEMATICS_H
#define CATCHROBO2023_ROBOT_KINEMATICS_H

//#include <Eigen>
#include "posrot_vector.h"

#define PI 3.14159265

const float robot_pos[6] = {675, -190, 0, 0, 0, 0};

class robot_kinematics {
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
     */
private:
    float link_len[5];//mm
    float joint_angle_lim[4][2];//rad, {inf, sup}

    void convert_field2robot(float*, float*);

public:
    robot_kinematics();
    // Public joint-angle order: [theta1, theta2, theta3, theta4] in radians.
    // theta2 and theta3 are absolute link angles referenced to the field frame.
    void inverse_kinematics(float*, float*);
    void forward_kinematics(float*, float*);
    void get_joint_positions(float *joint_angle, float positions[6][3]);
};

#endif //CATCHROBO2023_ROBOT_KINEMATICS_H
