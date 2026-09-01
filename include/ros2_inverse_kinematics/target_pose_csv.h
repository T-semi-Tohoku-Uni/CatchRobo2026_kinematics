#ifndef CATCHROBO2023_TARGET_POSE_CSV_H
#define CATCHROBO2023_TARGET_POSE_CSV_H

#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace catchrobo_kinematics {

constexpr std::size_t kExpectedTargetPoseCount = 24;

enum class TargetPoseKind {
    Pick,
    Shooting
};

enum class TeamSide {
    Red,
    Blue
};

inline const char *team_side_name(TeamSide team_side)
{
    return team_side == TeamSide::Red ? "red" : "blue";
}

struct TargetPose {
    int id;
    TeamSide team_side;
    int row_index;
    int column_index;
    int group_index;
    int position_index;
    double x_millimetres;
    double y_millimetres;
    double z_millimetres;
    double theta_radians;

    std::array<float, 6> inverse_kinematics_pose() const
    {
        return {{
            static_cast<float>(x_millimetres),
            static_cast<float>(y_millimetres),
            static_cast<float>(z_millimetres),
            static_cast<float>(theta_radians),
            0.0F,
            0.0F
        }};
    }

    std::array<double, 3> position_metres() const
    {
        return {{
            x_millimetres * 0.001,
            y_millimetres * 0.001,
            z_millimetres * 0.001
        }};
    }

    // Quaternion order matches geometry_msgs: x, y, z, w.
    std::array<double, 4> yaw_quaternion_xyzw() const
    {
        const double half_angle = theta_radians * 0.5;
        return {{0.0, 0.0, std::sin(half_angle), std::cos(half_angle)}};
    }

    // PoseMessage is intended to be geometry_msgs::msg::Pose. Keeping this a
    // template avoids adding a geometry_msgs dependency to the kinematics core.
    template<typename PoseMessage>
    void assign_ros_pose(PoseMessage &pose) const
    {
        const std::array<double, 3> position = position_metres();
        const std::array<double, 4> quaternion = yaw_quaternion_xyzw();
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.x = quaternion[0];
        pose.orientation.y = quaternion[1];
        pose.orientation.z = quaternion[2];
        pose.orientation.w = quaternion[3];
    }

    // Request is intended to be
    // catchrobo2026_msgs::srv::InverseKinematics::Request.
    template<typename Request>
    void assign_inverse_kinematics_request(Request &request) const
    {
        request.target_pose = inverse_kinematics_pose();
    }
};

namespace target_pose_csv_detail {

inline std::string trim(const std::string &value)
{
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return std::string();
    }
    const std::size_t last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1);
}

inline std::vector<std::string> split_csv_line(const std::string &line)
{
    std::vector<std::string> fields;
    std::string field;
    bool inside_quotes = false;

    for (std::size_t index = 0; index < line.size(); ++index) {
        const char character = line[index];
        if (character == '"') {
            if (inside_quotes && index + 1 < line.size() &&
                line[index + 1] == '"') {
                field.push_back('"');
                ++index;
            } else {
                inside_quotes = !inside_quotes;
            }
        } else if (character == ',' && !inside_quotes) {
            fields.push_back(trim(field));
            field.clear();
        } else {
            field.push_back(character);
        }
    }

    if (inside_quotes) {
        throw std::runtime_error("Unterminated quoted CSV field");
    }
    fields.push_back(trim(field));
    return fields;
}

inline std::vector<std::string> expected_header(TargetPoseKind kind)
{
    if (kind == TargetPoseKind::Pick) {
        return std::vector<std::string>{
            "ID", "TeamSide", "RowIdx", "ColIdx",
            "X", "Y", "Z", "Theta_rad"
        };
    }
    return std::vector<std::string>{
        "ID", "TeamSide", "GroupIdx", "PositionIdx",
        "RowIdx", "ColIdx", "X", "Y", "Z", "Theta_rad"
    };
}

inline int parse_integer(
    const std::string &text, const std::string &column, std::size_t line_number)
{
    std::size_t parsed = 0;
    long value = 0;
    try {
        value = std::stol(text, &parsed);
    } catch (const std::exception &) {
        throw std::runtime_error(
            "Line " + std::to_string(line_number) + " column " + column +
            " must be an integer");
    }
    if (parsed != text.size() || value < std::numeric_limits<int>::min() ||
        value > std::numeric_limits<int>::max()) {
        throw std::runtime_error(
            "Line " + std::to_string(line_number) + " column " + column +
            " must be an integer");
    }
    return static_cast<int>(value);
}

inline double parse_finite_number(
    const std::string &text, const std::string &column, std::size_t line_number)
{
    std::size_t parsed = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &parsed);
    } catch (const std::exception &) {
        throw std::runtime_error(
            "Line " + std::to_string(line_number) + " column " + column +
            " must be a finite number");
    }
    if (parsed != text.size() || !std::isfinite(value)) {
        throw std::runtime_error(
            "Line " + std::to_string(line_number) + " column " + column +
            " must be a finite number");
    }
    return value;
}

inline TeamSide parse_team_side(
    const std::string &text, std::size_t line_number)
{
    if (text == "red") {
        return TeamSide::Red;
    }
    if (text == "blue") {
        return TeamSide::Blue;
    }
    throw std::runtime_error(
        "Line " + std::to_string(line_number) +
        " TeamSide must be red or blue");
}

}  // namespace target_pose_csv_detail

inline std::vector<TargetPose> load_target_pose_csv(
    const std::string &path,
    TargetPoseKind kind,
    TeamSide expected_team_side,
    std::size_t expected_count = kExpectedTargetPoseCount)
{
    std::ifstream stream(path.c_str());
    if (!stream) {
        throw std::runtime_error("Could not open target pose CSV: " + path);
    }

    std::string line;
    if (!std::getline(stream, line)) {
        throw std::runtime_error("Target pose CSV is empty: " + path);
    }

    std::vector<std::string> header =
        target_pose_csv_detail::split_csv_line(line);
    if (!header.empty() && header[0].size() >= 3 &&
        static_cast<unsigned char>(header[0][0]) == 0xEF &&
        static_cast<unsigned char>(header[0][1]) == 0xBB &&
        static_cast<unsigned char>(header[0][2]) == 0xBF) {
        header[0].erase(0, 3);
    }
    if (header != target_pose_csv_detail::expected_header(kind)) {
        throw std::runtime_error("Unexpected CSV header in: " + path);
    }

    std::vector<TargetPose> poses;
    std::set<int> identifiers;
    std::size_t line_number = 1;

    while (std::getline(stream, line)) {
        ++line_number;
        if (target_pose_csv_detail::trim(line).empty()) {
            continue;
        }

        const std::vector<std::string> fields =
            target_pose_csv_detail::split_csv_line(line);
        if (fields.size() != header.size()) {
            throw std::runtime_error(
                "Line " + std::to_string(line_number) +
                " has the wrong number of columns");
        }

        TargetPose pose{};
        pose.id = target_pose_csv_detail::parse_integer(
            fields[0], "ID", line_number);
        pose.team_side = target_pose_csv_detail::parse_team_side(
            fields[1], line_number);
        if (pose.team_side != expected_team_side) {
            throw std::runtime_error(
                "Line " + std::to_string(line_number) +
                " TeamSide does not match the selected team");
        }
        if (!identifiers.insert(pose.id).second) {
            throw std::runtime_error(
                "Duplicate ID " + std::to_string(pose.id));
        }

        std::size_t coordinate_index = 0;
        if (kind == TargetPoseKind::Pick) {
            pose.row_index = target_pose_csv_detail::parse_integer(
                fields[2], "RowIdx", line_number);
            pose.column_index = target_pose_csv_detail::parse_integer(
                fields[3], "ColIdx", line_number);
            pose.group_index = 0;
            pose.position_index = 0;
            coordinate_index = 4;
        } else {
            pose.group_index = target_pose_csv_detail::parse_integer(
                fields[2], "GroupIdx", line_number);
            pose.position_index = target_pose_csv_detail::parse_integer(
                fields[3], "PositionIdx", line_number);
            pose.row_index = target_pose_csv_detail::parse_integer(
                fields[4], "RowIdx", line_number);
            pose.column_index = target_pose_csv_detail::parse_integer(
                fields[5], "ColIdx", line_number);
            coordinate_index = 6;
        }

        pose.x_millimetres = target_pose_csv_detail::parse_finite_number(
            fields[coordinate_index], "X", line_number);
        pose.y_millimetres = target_pose_csv_detail::parse_finite_number(
            fields[coordinate_index + 1], "Y", line_number);
        pose.z_millimetres = target_pose_csv_detail::parse_finite_number(
            fields[coordinate_index + 2], "Z", line_number);
        pose.theta_radians = target_pose_csv_detail::parse_finite_number(
            fields[coordinate_index + 3], "Theta_rad", line_number);
        poses.push_back(pose);
    }

    if (expected_count != 0 && poses.size() != expected_count) {
        throw std::runtime_error(
            "Expected " + std::to_string(expected_count) + " rows in " +
            path + ", got " + std::to_string(poses.size()));
    }
    return poses;
}

inline const TargetPose &find_target_by_id(
    const std::vector<TargetPose> &poses, int id)
{
    for (std::size_t index = 0; index < poses.size(); ++index) {
        if (poses[index].id == id) {
            return poses[index];
        }
    }
    throw std::out_of_range("Target pose ID not found: " + std::to_string(id));
}

}  // namespace catchrobo_kinematics

#endif  // CATCHROBO2023_TARGET_POSE_CSV_H
