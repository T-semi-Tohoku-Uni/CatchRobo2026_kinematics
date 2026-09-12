# CatchRobo 2026 kinematics v0.1.1

CatchRobo 2026 の4自由度多重平行リンクアーム向けROS 2運動学パッケージです。
既存のROSノード、共有ライブラリ、サービスAPI、可視化用座標変換に加え、
入力検査と倍精度の内部計算を提供します。

## Geometry and conventions

- フィールド座標での機体原点: `[675, -190, 0]` mm
- 上腕・前腕: 各480 mm
- フランジの水平オフセット: 40 mm
- ベース径方向オフセット: -40 mm
- ベース高さオフセット: 90 mm
- ツール垂直オフセット: 0 mm
- `theta2` と `theta3`: フィールドZ+を0とする絶対リンク角
- フランジyaw: `phi = theta1 + theta4`
- `theta1`: フィールドY+を0とし、`atan2(x, y)` の主値
  `[-pi, pi]` を返す
- 肩角: `atan2(rho, z)` を使う旧統合版互換の分岐

v0.1.0では機体原点、ベースオフセット、`theta1` の符号、肩角分岐を変更したため、
同じ目標姿勢から従来と異なる4軸指令を生成していました。v0.1.1はこれらを
旧統合版 `b3760d7` の規約へ戻し、旧統合版の関節指令規約との互換性を復旧しています。
幾何定数と変換規約の正本は
`include/ros2_inverse_kinematics/homogeneous_transform.h` です。

非有限入力、到達不能姿勢、完全折畳み姿勢は全4軸NaNとして拒否します。
Humble、実CAN、実機での関節速度、干渉、寸法の妥当性は未検証です。

同梱の `config/joint_targets/*.csv` は `tools/generate_joint_targets.cpp` で
`config/flange_targets/*.csv` から生成します。寸法や角度規約を変更した場合は
再生成してください。
