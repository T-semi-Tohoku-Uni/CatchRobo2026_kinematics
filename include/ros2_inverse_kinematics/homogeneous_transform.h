#ifndef CATCHROBO2023_HOMOGENEOUS_TRANSFORM_H
#define CATCHROBO2023_HOMOGENEOUS_TRANSFORM_H

#include <array>
#include <cmath>

namespace catchrobo_kinematics {

using TransformMatrix = std::array<double, 16>;
using TransformChain = std::array<TransformMatrix, 6>;
using JointAngles = std::array<double, 4>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRobotXMillimetres = 675.0;
constexpr double kRobotYMillimetres = -190.0;
constexpr double kRobotZMillimetres = 0.0;
constexpr double kBaseRadialOffsetMillimetres = -40.0;
constexpr double kBaseHeightMillimetres = 90.0;
constexpr double kUpperArmLengthMillimetres = 480.0;
constexpr double kForearmLengthMillimetres = 480.0;
constexpr double kFlangeOffsetMillimetres = 40.0;

inline TransformMatrix identity_transform()
{
    return {{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    }};
}

inline TransformMatrix multiply(
    const TransformMatrix &left, const TransformMatrix &right)
{
    TransformMatrix result = {{}};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            for (int inner = 0; inner < 4; ++inner) {
                result[row * 4 + column] +=
                    left[row * 4 + inner] * right[inner * 4 + column];
            }
        }
    }
    return result;
}

inline TransformMatrix translation(double x, double y, double z)
{
    TransformMatrix result = identity_transform();
    result[3] = x;
    result[7] = y;
    result[11] = z;
    return result;
}

inline TransformMatrix rotation_x(double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{
        1.0, 0.0, 0.0, 0.0,
        0.0, cosine, -sine, 0.0,
        0.0, sine, cosine, 0.0,
        0.0, 0.0, 0.0, 1.0
    }};
}

inline TransformMatrix rotation_y(double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{
        cosine, 0.0, sine, 0.0,
        0.0, 1.0, 0.0, 0.0,
        -sine, 0.0, cosine, 0.0,
        0.0, 0.0, 0.0, 1.0
    }};
}

inline TransformMatrix rotation_z(double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{
        cosine, -sine, 0.0, 0.0,
        sine, cosine, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    }};
}

inline TransformMatrix pose_with_rotation(
    const TransformMatrix &pose, const TransformMatrix &rotation)
{
    TransformMatrix result = rotation;
    result[3] = pose[3];
    result[7] = pose[7];
    result[11] = pose[11];
    return result;
}

// The public interface uses field-referenced (absolute) link angles, while the
// serial kinematic chain uses joint-to-joint (relative) rotations.
// Angle order: [theta1, theta2, theta3, theta4].
inline JointAngles absolute_to_relative_joint_angles(
    const JointAngles &absolute_joint_angles)
{
    JointAngles relative_joint_angles = absolute_joint_angles;
    relative_joint_angles[2] =
        absolute_joint_angles[2] - absolute_joint_angles[1];
    return relative_joint_angles;
}

inline JointAngles relative_to_absolute_joint_angles(
    const JointAngles &relative_joint_angles)
{
    JointAngles absolute_joint_angles = relative_joint_angles;
    absolute_joint_angles[2] =
        relative_joint_angles[1] + relative_joint_angles[2];
    return absolute_joint_angles;
}

inline double dependent_theta2_prime(
    double theta2_relative, double theta3_relative)
{
    return -kPi / 2.0 - theta2_relative - theta3_relative;
}

// relative_joint_angles: [theta1, theta2_relative, theta3_relative, theta4]
//
// The IK convention is authoritative here:
//   theta1 = atan2(robot_x, robot_y)
//   theta2_relative is measured from field Z+ in the two-link arm plane
//   theta3_relative is measured from the upper arm
//   theta2_prime + theta2_relative + theta3_relative = -pi/2
//
// frame[0]: field -> robot base
// frame[1]: field -> theta1 frame
// frame[2]: field -> theta2 frame
// frame[3]: field -> theta3 frame
// frame[4]: field -> theta2-prime frame
// frame[5]: field -> theta4/flange frame
inline TransformChain make_transform_chain(
    const JointAngles &relative_joint_angles)
{
    const double theta1 = relative_joint_angles[0];
    const double theta2 = relative_joint_angles[1];
    const double theta3 = relative_joint_angles[2];
    const double theta4 = relative_joint_angles[3];
    const double theta2_prime = dependent_theta2_prime(theta2, theta3);

    TransformChain frame;
    frame[0] = translation(
        kRobotXMillimetres,
        kRobotYMillimetres,
        kRobotZMillimetres);

    // theta1 turns the arm plane toward the target.  theta2 and theta3 are
    // zero when their links point along field Z+ and increase toward the
    // radial direction in that plane.
    frame[1] = multiply(frame[0], rotation_z(kPi / 2.0 - theta1));
    frame[2] = multiply(frame[1], multiply(
        translation(kBaseRadialOffsetMillimetres, 0.0,
            kBaseHeightMillimetres),
        rotation_y(theta2)));
    frame[3] = multiply(frame[2], multiply(
        translation(0.0, 0.0, kUpperArmLengthMillimetres),
        rotation_y(theta3)));
    frame[4] = multiply(frame[3], multiply(
        translation(0.0, 0.0, kForearmLengthMillimetres),
        rotation_y(theta2_prime)));
    const TransformMatrix flange_position = multiply(
        frame[4], translation(kFlangeOffsetMillimetres, 0.0, 0.0));

    // The dependent joint keeps field-frame roll and pitch fixed.  The only
    // controllable flange orientation is field-frame yaw, whose IK convention
    // is phi = theta1 + theta4.  Position is independent of theta4 because the
    // flange point and the theta4 rotation point coincide.
    frame[5] = pose_with_rotation(
        flange_position, rotation_z(theta1 + theta4));

    return frame;
}

}  // namespace catchrobo_kinematics

#endif  // CATCHROBO2023_HOMOGENEOUS_TRANSFORM_H
