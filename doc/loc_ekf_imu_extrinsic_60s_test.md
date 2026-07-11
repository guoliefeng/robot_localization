# loc_ekf_ins_imu_wheel IMU 外参 60 秒排查记录

测试日期：2026-07-07

## 目标

排查 `loc_ekf_ins_imu_wheel_node` 中 IMU 外参是否可能导致横向误差。测试只使用数据集前 60 秒，并用 `rosbag play -r 3 -u 60` 加速回放。

## 代码检查结论

当前节点默认只融合 IMU yaw rate：

- `imu_use_orientation: false`
- `imu_use_angular_velocity: true`
- `imu_use_linear_acceleration: false`

因此当前真正进入 EKF 的 IMU 信息是旋转后的 `angular_velocity.z`。如果外参只改变 yaw，例如 `-90 deg / 0 deg / +90 deg`，绕 z 轴角速度的 z 分量不会改变，所以 yaw 外参本身不应造成明显横向误差。

本次修正了 launch 的一个可调参问题：

- `launch/loc_ekf_ins_imu_wheel.launch` 新增 `imu_extrinsic_roll`
- `launch/loc_ekf_ins_imu_wheel.launch` 新增 `imu_extrinsic_pitch`

这样 roll/pitch/yaw 三个外参都可以从 launch 命令传入。

## 原始数据轴向检查

数据：

```text
/home/guoli/data/yangpu/lcp/0706/204-ok/223_7_6_0.bag
```

统计前 60 秒 IMU 三轴角速度与 INS/LIO/wheel yaw rate 的相关性：

| 参考 yaw rate | IMU wx corr | IMU wy corr | IMU wz corr | 结论 |
| --- | ---: | ---: | ---: | --- |
| `/localization/ins` | -0.025 | -0.015 | 0.898 | yaw rate 在 IMU z 轴 |
| `/lio_loc_result` | -0.008 | -0.024 | 0.914 | yaw rate 在 IMU z 轴 |
| `/wheel_odom` | -0.011 | -0.024 | 0.999 | yaw rate 在 IMU z 轴 |

结论：IMU yaw rate 轴向是 z 轴，符号为正，不应做 roll=180 或 pitch=180 这种会翻转 z 轴的外参。

## 节点测试配置

统一启动：

```bash
roslaunch localization_switcher analysis.launch record:=false start_rviz:=false
roslaunch robot_loc loc_ekf_ins_imu_wheel.launch \
  test_drop_ins_enabled:=true \
  test_drop_ins_period_sec:=1000 \
  test_drop_ins_duration_sec:=20
rosbag play /home/guoli/data/yangpu/lcp/0706/204-ok/223_7_6_* --clock -r 3 -u 60
```

记录目录：

```text
/tmp/loc_ekf_imu_extrinsic_60s
```

误差统计均以 `/localization/ins` 为参考，并把位置误差拆成：

- `along`：沿车体前向误差
- `lat`：车体横向误差
- `yaw`：航向误差

## 第一轮：20-40 秒人工丢 INS

| case | 外参/配置 | pos mean | pos p95 | pos max | lat mean | lat p95 | lat max | yaw mean | yaw p95 | 恢复后 pos mean |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| current_yaw_m90 | yaw=-90 deg | 0.342 m | 0.376 m | 0.572 m | 0.088 m | 0.242 m | 0.290 m | 0.078 deg | 0.234 deg | 0.177 m |
| yaw_0 | yaw=0 deg | 0.295 m | 0.347 m | 0.516 m | 0.147 m | 0.276 m | 0.279 m | 0.222 deg | 0.549 deg | 0.012 m |
| yaw_p90 | yaw=+90 deg | 0.303 m | 0.355 m | 0.517 m | 0.148 m | 0.281 m | 0.284 m | 0.214 deg | 0.541 deg | 0.021 m |
| imu_off | 不融合 IMU angular velocity | 0.254 m | 0.320 m | 0.424 m | 0.127 m | 0.239 m | 0.402 m | 0.434 deg | 1.233 deg | 0.006 m |
| roll_pi | roll=180 deg，翻转 z 轴 | 6.138 m | 19.511 m | 22.345 m | 2.802 m | 7.151 m | 7.226 m | 68.138 deg | 170.660 deg | 2.158 m |

观察：

- yaw=-90、yaw=0、yaw=+90 三组位置/横向误差同量级。
- roll=180 明显错误，yaw 和位置都快速发散。
- IMU 关闭时位置误差不一定更差，但 yaw 误差明显变大。

## 第二轮：30-50 秒人工丢 INS

| case | 外参/配置 | pos mean | pos p95 | pos max | lat mean | lat p95 | lat max | yaw mean | yaw p95 | 恢复后 pos mean |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| current_yaw_m90_b | yaw=-90 deg | 0.463 m | 0.636 m | 0.960 m | 0.285 m | 0.508 m | 0.521 m | 0.646 deg | 0.779 deg | 0.016 m |
| yaw_0_b | yaw=0 deg | 0.465 m | 0.657 m | 0.874 m | 0.300 m | 0.538 m | 0.560 m | 0.632 deg | 0.752 deg | 0.016 m |
| imu_off_b | 不融合 IMU angular velocity | 0.910 m | 1.816 m | 1.915 m | 0.644 m | 1.699 m | 1.799 m | 4.105 deg | 4.562 deg | 0.006 m |
| roll_pi_b | roll=180 deg，翻转 z 轴 | 12.035 m | 28.129 m | 30.514 m | 4.441 m | 12.611 m | 13.945 m | 88.543 deg | 112.153 deg | 0.017 m |

观察：

- current yaw=-90 与 yaw=0 几乎一致，说明当前横向误差不是 yaw 外参导致的。
- 不用 IMU 时，yaw 误差和横向误差都明显变大；这一段 IMU 是有帮助的。
- z 轴翻转会严重发散，证明 IMU z 轴符号必须保持当前方向。

## 结论

1. 当前数据中 IMU yaw rate 的有效轴是 `angular_velocity.z`，符号为正。
2. 当前节点只融合 `angular_velocity.z`，所以 yaw 外参 `-90/0/+90 deg` 对结果影响很小；横向误差不太可能由 yaw 外参引起。
3. roll/pitch 如果导致 z 轴翻转，会立刻造成 yaw 发散和横向漂移；测试中的 `roll=180 deg` 明显不正常。
4. IMU 在 30-50 秒人工丢 INS 窗口内有正向作用：关闭 IMU 后横向 p95 从约 0.51-0.54 m 增大到 1.70 m。
5. 当前观察到的横向误差更可能来自 wheel odom 的尺度、轮速坐标系/速度分解、或无绝对约束 dead reckoning 累积，而不是 IMU yaw 外参。

建议当前先保持：

```text
imu_extrinsic_roll: 0.0
imu_extrinsic_pitch: 0.0
imu_extrinsic_yaw: -1.57079632679
imu_use_angular_velocity: true
imu_use_orientation: false
imu_use_linear_acceleration: false
```

如果后续要融合 IMU orientation 或 linear acceleration，再重新标定完整 `R_base_imu`，因为那时 yaw/roll/pitch 外参都会直接影响 EKF。
