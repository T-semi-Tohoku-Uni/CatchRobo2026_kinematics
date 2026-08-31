# Joint target reference CSVs

These four CSV files are generated from the flange targets in
`../flange_targets` using the current `robot_kinematics::inverse_kinematics`
implementation.

- `pick_joint_targets_red.csv`
- `pick_joint_targets_blue.csv`
- `shooting_joint_targets_red.csv`
- `shooting_joint_targets_blue.csv`

The source pose columns are kept in each row for traceability. The generated
joint columns are expressed in radians:

- `Theta1_rad`
- `Theta2_rad`
- `Theta3_rad`
- `Theta2Prime_rad`
- `Theta4_rad`

`Theta2Prime_rad` is dependent and satisfies:

```text
Theta2Prime_rad + Theta2_rad + Theta3_rad = -pi/2
```

These are reference outputs for the current IK convention. Regenerate and
review them whenever the robot geometry, coordinate convention, or IK branch
selection changes.

From WSL2, regenerate with:

```bash
cd /mnt/c/Users/YYcri/Documents/T-semi/catchrobo2026/catch-robo-2026/src/CatchRobo2026_kinematics
g++ -std=c++11 -Wall -Wextra -Wpedantic -Iinclude \
  src/robot_kinematics.cpp tools/generate_joint_targets.cpp \
  -o /tmp/generate_joint_targets
/tmp/generate_joint_targets config/flange_targets config/joint_targets
```
