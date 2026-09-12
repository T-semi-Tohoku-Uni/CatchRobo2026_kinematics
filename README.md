# CatchRobo 2026 kinematics v0.1.0

CatchRobo 2026 の4自由度多重平行リンクアーム向けROS 2運動学パッケージです。
新版の運動学（upstream `master` の `2d1514f`）にある機体原点とリンク寸法を、
既存のROSノード、共有ライブラリ、サービスAPI、可視化用座標変換へ統合しています。

## Geometry and conventions

- フィールド座標での機体原点: `[675, -130, 228]` mm
- 上腕・前腕: 各480 mm
- フランジの水平オフセット: 40 mm
- ベース径方向・高さオフセット、ツール垂直オフセット: 0 mm
- `theta2` と `theta3`: フィールドZ+を0とする絶対リンク角
- フランジyaw: `phi = theta1 + theta4`
- `theta1`: フィールドY+を0、反時計回りを正とし、`atan2(-x, y)` の主値
  `[-pi, pi]` を返す
- 肩角: `pi/2 - atan2(z, rho)` を使う新版分岐

旧統合版から機体原点、ベースオフセット、`theta1` の符号と主値、肩角分岐を
変更しました。`theta1` の主値は負X側の経路、肩角分岐はフランジ半径境界で
不要な2πジャンプが生じないようにしています。幾何定数と変換規約の正本は
`include/ros2_inverse_kinematics/homogeneous_transform.h` です。

既存ending経路は完全折畳み姿勢から直線評価で約2.8 mmの近傍を通ります。
実機での関節速度、干渉、CAN軸方向、寸法の妥当性は未検証です。

同梱の `config/joint_targets/*.csv` は `tools/generate_joint_targets.cpp` で
`config/flange_targets/*.csv` から生成します。寸法や角度規約を変更した場合は
再生成してください。
