#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ros2_inverse_kinematics/homogeneous_transform.h"
#include "ros2_inverse_kinematics/robot_kinematics.h"
#include "ros2_inverse_kinematics/target_pose_csv.h"

namespace {

struct FileSpecification {
    const char *input_filename;
    const char *output_filename;
    catchrobo_kinematics::TargetPoseKind kind;
    catchrobo_kinematics::TeamSide team_side;
};

bool all_finite(const float *joint_angles)
{
    for (int index = 0; index < 4; ++index) {
        if (!std::isfinite(joint_angles[index])) {
            return false;
        }
    }
    return true;
}

void write_header(
    std::ofstream &stream, catchrobo_kinematics::TargetPoseKind kind)
{
    if (kind == catchrobo_kinematics::TargetPoseKind::Pick) {
        stream << "ID,TeamSide,RowIdx,ColIdx";
    } else {
        stream << "ID,TeamSide,GroupIdx,PositionIdx,RowIdx,ColIdx";
    }
    stream << ",X,Y,Z,Theta_rad"
           << ",Theta1_rad,Theta2_rad,Theta3_rad"
           << ",Theta2Prime_rad,Theta4_rad\n";
}

void write_target_metadata(
    std::ofstream &stream,
    const catchrobo_kinematics::TargetPose &target,
    catchrobo_kinematics::TargetPoseKind kind)
{
    stream << target.id << ','
           << catchrobo_kinematics::team_side_name(target.team_side) << ',';
    if (kind == catchrobo_kinematics::TargetPoseKind::Pick) {
        stream << target.row_index << ',' << target.column_index;
    } else {
        stream << target.group_index << ',' << target.position_index << ','
               << target.row_index << ',' << target.column_index;
    }
    stream << ',' << target.x_millimetres
           << ',' << target.y_millimetres
           << ',' << target.z_millimetres
           << ',' << target.theta_radians;
}

void generate_file(
    const std::string &input_directory,
    const std::string &output_directory,
    const FileSpecification &specification)
{
    const std::string input_path =
        input_directory + "/" + specification.input_filename;
    const std::string output_path =
        output_directory + "/" + specification.output_filename;

    const std::vector<catchrobo_kinematics::TargetPose> targets =
        catchrobo_kinematics::load_target_pose_csv(
            input_path, specification.kind, specification.team_side);

    std::vector<std::array<float, 5> > joint_rows;
    joint_rows.reserve(targets.size());
    robot_kinematics kinematics;

    for (std::size_t index = 0; index < targets.size(); ++index) {
        std::array<float, 6> pose = targets[index].inverse_kinematics_pose();
        float independent_joint_angles[4] = {};
        kinematics.inverse_kinematics(
            pose.data(), independent_joint_angles);
        if (!all_finite(independent_joint_angles)) {
            throw std::runtime_error(
                std::string(specification.input_filename) + " ID=" +
                std::to_string(targets[index].id) + " is unreachable");
        }

        const catchrobo_kinematics::JointAngles absolute_joint_angles = {{
            independent_joint_angles[0], independent_joint_angles[1],
            independent_joint_angles[2], independent_joint_angles[3]
        }};
        const catchrobo_kinematics::JointAngles relative_joint_angles =
            catchrobo_kinematics::absolute_to_relative_joint_angles(
                absolute_joint_angles);
        const float theta2_prime = static_cast<float>(
            catchrobo_kinematics::dependent_theta2_prime(
                relative_joint_angles[1], relative_joint_angles[2]));
        joint_rows.push_back({{
            independent_joint_angles[0],
            independent_joint_angles[1],
            independent_joint_angles[2],
            theta2_prime,
            independent_joint_angles[3]
        }});
    }

    std::ofstream output(output_path.c_str());
    if (!output) {
        throw std::runtime_error("Could not create: " + output_path);
    }
    output << std::setprecision(15);
    write_header(output, specification.kind);
    for (std::size_t index = 0; index < targets.size(); ++index) {
        write_target_metadata(output, targets[index], specification.kind);
        for (std::size_t joint = 0; joint < joint_rows[index].size(); ++joint) {
            output << ',' << joint_rows[index][joint];
        }
        output << '\n';
    }
    if (!output) {
        throw std::runtime_error("Failed while writing: " + output_path);
    }

    std::cout << specification.output_filename
              << ": wrote " << targets.size() << " rows\n";
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc != 3) {
        std::cerr << "Usage: generate_joint_targets "
                  << "<flange-target-directory> <output-directory>\n";
        return 2;
    }

    const std::array<FileSpecification, 4> specifications = {{
        {"pick_pose_red.csv", "pick_joint_targets_red.csv",
            catchrobo_kinematics::TargetPoseKind::Pick,
            catchrobo_kinematics::TeamSide::Red},
        {"pick_pose_blue.csv", "pick_joint_targets_blue.csv",
            catchrobo_kinematics::TargetPoseKind::Pick,
            catchrobo_kinematics::TeamSide::Blue},
        {"shooting_pose_red.csv", "shooting_joint_targets_red.csv",
            catchrobo_kinematics::TargetPoseKind::Shooting,
            catchrobo_kinematics::TeamSide::Red},
        {"shooting_pose_blue.csv", "shooting_joint_targets_blue.csv",
            catchrobo_kinematics::TargetPoseKind::Shooting,
            catchrobo_kinematics::TeamSide::Blue}
    }};

    try {
        for (std::size_t index = 0; index < specifications.size(); ++index) {
            generate_file(argv[1], argv[2], specifications[index]);
        }
    } catch (const std::exception &error) {
        std::cerr << "Joint-target generation failed: "
                  << error.what() << '\n';
        return 1;
    }

    return 0;
}
