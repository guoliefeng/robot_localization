# loc_ekf_ins_imu_wheel IMU 线加速度 60 秒测试记录

测试日期：2026-07-07

## 背景

`/ins_driver/imu` 的 `linear_acceleration` 原始单位是 g，不是 `m/s^2`。节点已增加：

```text
imu_linear_acceleration_scale: 9.806
```

用于在送入 EKF 前把加速度从 g 转成 `m/s^2`。

当前默认仍然是：

```text
imu_use_linear_acceleration: false
```

即默认只融合 IMU yaw rate，不融合 IMU 线加速度。

## 测试方法

只测试数据集前 60 秒：

```bash
roslaunch localization_switcher analysis.launch record:=false start_rviz:=false
roslaunch robot_loc loc_ekf_ins_imu_wheel.launch \
  test_drop_ins_enabled:=true \
  test_drop_ins_period_sec:=1000 \
  ...
rosbag play /home/guoli/data/yangpu/lcp/0706/204-ok/223_7_6_* --clock -r 3 -u 60
```

记录目录：

```text
/tmp/loc_ekf_accel_60s
/tmp/loc_ekf_accel_60s_b
```

误差参考：`/localization/ins`。

## 第一轮：30-50 秒人工丢 INS

| case | 配置 | pos mean | pos p95 | pos max | lat mean | lat p95 | lat max | yaw mean | yaw p95 | 恢复后 pos mean |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| yaw_only | 不融合线加速度 | 4.469 m | 7.017 m | 7.528 m | 1.782 m | 4.980 m | 5.546 m | 25.236 deg | 25.747 deg | 0.016 m |
| accel_std1 | scale=9.806, acc std=1 | 0.971 m | 1.887 m | 3.970 m | 0.805 m | 1.876 m | 3.968 m | 0.443 deg | 0.620 deg | 0.352 m |
| accel_std5 | scale=9.806, acc std=5 | 0.541 m | 1.543 m | 1.950 m | 0.379 m | 1.489 m | 1.906 m | 0.629 deg | 0.770 deg | 0.016 m |
| accel_std10 | scale=9.806, acc std=10 | 0.496 m | 0.961 m | 0.994 m | 0.355 m | 0.892 m | 0.926 m | 0.481 deg | 0.615 deg | 0.015 m |
| accel_wrong_scale1 | scale=1.0, acc std=5 | 0.466 m | 0.748 m | 0.901 m | 0.312 m | 0.642 m | 0.674 m | 0.733 deg | 0.873 deg | 0.016 m |

说明：本轮 baseline 在人工丢失前附近遇到过上游 `LOST`，因此 yaw_only 出现明显发散。线加速度在这一轮能抑制发散，但该窗口不够干净，不能直接作为默认打开线加速度的依据。

## 第二轮：40-55 秒人工丢 INS

| case | 配置 | pos mean | pos p95 | pos max | lat mean | lat p95 | lat max | yaw mean | yaw p95 | 恢复后 pos mean |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| yaw_only | 不融合线加速度 | 0.751 m | 0.826 m | 1.172 m | 0.229 m | 0.274 m | 0.277 m | 0.064 deg | 0.087 deg | 0.015 m |
| accel_std5 | scale=9.806, acc std=5 | 0.949 m | 1.274 m | 1.511 m | 0.486 m | 0.862 m | 1.250 m | 0.045 deg | 0.065 deg | 0.024 m |
| accel_std10 | scale=9.806, acc std=10 | 0.772 m | 0.852 m | 1.156 m | 0.255 m | 0.362 m | 0.369 m | 0.047 deg | 0.066 deg | 无完整恢复窗口 |
| accel_std20 | scale=9.806, acc std=20 | 0.751 m | 0.840 m | 1.123 m | 0.261 m | 0.350 m | 0.386 m | 0.044 deg | 0.068 deg | 0.015 m |
| accel_wrong_scale1 | scale=1.0, acc std=5 | 0.788 m | 0.886 m | 1.155 m | 0.278 m | 0.386 m | 0.425 m | 0.055 deg | 0.081 deg | 0.016 m |

## 结论

1. `imu_linear_acceleration_scale=9.806` 是必要的单位修正；如果后续打开线加速度，不能直接使用原始 g 值。
2. 在线加速度融合较强时，例如 `std=5`，横向误差会变大，说明 IMU 加速度仍受到重力补偿、零偏和外参误差影响。
3. 在线加速度很弱时，例如 `std=20`，效果与 yaw-only 基本一致，风险较低。
4. 线加速度在某些发散窗口能帮忙抑制漂移，但在稳定窗口没有形成一致收益。

## 最终参数选择

保持默认不融合线加速度：

```text
imu_use_linear_acceleration: false
```

同时把线加速度默认 covariance 调弱：

```text
imu_primary_linear_acceleration_std: 20.0
imu_weak_linear_acceleration_std: 20.0
imu_degraded_linear_acceleration_std: 20.0
```

这样如果现场临时打开：

```bash
imu_use_linear_acceleration:=true
```

线加速度只会作为很弱的约束，不会强行拉动 final_odom。
