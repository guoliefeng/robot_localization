# loc_ekf_ins_imu_wheel 10 秒 INS 丢失测试记录

测试日期：2026-07-07

## 测试目标

验证 `loc_ekf_ins_imu_wheel_node` 在 INS 人工丢失 10 秒时，是否能通过 IMU + wheel odom 继续输出 `/localization/final_odom`，并在 INS 恢复后回到 INS 坐标。

## 测试环境

- 上游数据：
  `/home/guoli/data/yangpu/lcp/0706/204-ok/223_7_6_*`
- 上游启动：
  `roslaunch localization_switcher analysis.launch record:=false start_rviz:=false`
- EKF 启动：
  `roslaunch robot_localization loc_ekf_ins_imu_wheel.launch test_drop_ins_enabled:=true ...`
- 回放：
  `rosbag play /home/guoli/data/yangpu/lcp/0706/204-ok/223_7_6_* --clock -r 3.0`
- 记录 topic：
  `/clock`
  `/localization/final_odom`
  `/localization/ins`
  `/lio_loc_result`
  `/wheel_odom`
  `/ins_driver/imu`
  `/localization/status`
  `/localization/loc_policy_code`

说明：本次为 3x 离线回放，因此 `/localization/final_odom` 记录频率约 30 Hz，不能作为真实 100 Hz 性能结论；本记录重点看丢失期间是否连续、漂移量和恢复情况。

## 测试一：20 秒开始，每 30 秒丢 INS 10 秒

参数：

```bash
test_drop_ins_start_sec:=20
test_drop_ins_period_sec:=30
test_drop_ins_duration_sec:=10
```

输出 bag：

```text
/tmp/loc_ekf_ins_imu_wheel_exp/drop_ins_10s.bag
```

日志确认：

- 进入人工丢失窗口后：`ins=DROP wheel=PRIMARY imu=PRIMARY test_drop_ins=1`
- 退出人工丢失窗口后：`ins=PRIMARY wheel=WEAK imu=WEAK test_drop_ins=0`

整体结果：

- `/localization/final_odom`：6255 帧
- 平均记录频率：30.51 Hz
- 相对 `/localization/ins`：
  - mean：0.147 m
  - p95：0.584 m
  - max：4.594 m
- 相对 `/lio_loc_result`：
  - mean：0.252 m
  - p95：0.563 m
  - max：4.589 m

10 秒人工丢失窗口，相对 INS 的平面误差：

| 窗口 | 相对时间 | mean | p95 | max | 恢复后 2 秒 mean | 恢复后 2 秒 max |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 20-30 s | 2.415 m | 2.546 m | 4.594 m | 0.008 m | 0.020 m |
| 2 | 50-60 s | 0.331 m | 0.737 m | 0.806 m | 0.015 m | 0.036 m |
| 3 | 80-90 s | 0.051 m | 0.082 m | 0.094 m | 0.018 m | 0.039 m |
| 4 | 110-120 s | 0.057 m | 0.105 m | 0.114 m | 0.011 m | 0.011 m |
| 5 | 140-150 s | 0.032 m | 0.060 m | 0.069 m | 0.014 m | 0.048 m |
| 6 | 170-180 s | 0.054 m | 0.083 m | 0.099 m | 0.015 m | 0.034 m |
| 7 | 200-205.4 s | 0.009 m | 0.011 m | 0.014 m | 无完整恢复窗口 | 无完整恢复窗口 |

结论：

第 1 个窗口漂移偏大，原因是窗口开始附近上游 Loc 状态处于 `LOST/NOT_STABLE`，节点策略曾短时把 INS、wheel、IMU 都 drop 掉，导致 final_odom 只能短时外推或无更新。这个窗口不代表稳定状态下人工丢 INS 的典型表现。

## 测试二：50 秒开始，每 30 秒丢 INS 10 秒

参数：

```bash
test_drop_ins_start_sec:=50
test_drop_ins_period_sec:=30
test_drop_ins_duration_sec:=10
```

输出 bag：

```text
/tmp/loc_ekf_ins_imu_wheel_exp/drop_ins_10s_stable.bag
```

日志确认：

- 进入人工丢失窗口后：`ins=DROP wheel=PRIMARY imu=PRIMARY test_drop_ins=1`
- 退出人工丢失窗口后：`ins=PRIMARY wheel=WEAK imu=WEAK test_drop_ins=0`

整体结果：

- `/localization/final_odom`：6283 帧
- 平均记录频率：30.70 Hz
- 相对 `/localization/ins`：
  - mean：0.146 m
  - p95：1.049 m
  - max：5.393 m
- 相对 `/lio_loc_result`：
  - mean：0.241 m
  - p95：1.024 m
  - max：5.406 m

10 秒人工丢失窗口，相对 INS 的平面误差：

| 窗口 | 相对时间 | mean | p95 | max | 恢复后 2 秒 mean | 恢复后 2 秒 max |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 50-60 s | 1.594 m | 2.390 m | 3.090 m | 0.018 m | 0.048 m |
| 2 | 80-90 s | 0.057 m | 0.085 m | 0.093 m | 0.014 m | 0.043 m |
| 3 | 110-120 s | 0.116 m | 0.231 m | 0.246 m | 0.015 m | 0.037 m |
| 4 | 140-150 s | 0.033 m | 0.061 m | 0.074 m | 0.015 m | 0.049 m |
| 5 | 170-180 s | 0.427 m | 0.965 m | 1.050 m | 0.014 m | 0.035 m |
| 6 | 200-205.1 s | 0.004 m | 0.008 m | 0.009 m | 无完整恢复窗口 | 无完整恢复窗口 |

补充观察：

- 人工丢 INS 窗口内 final_odom 没有中断，窗口内输出间隔最大约 0.05 s。
- INS 恢复后，2 秒内 final_odom 基本回到 INS 附近，恢复后 mean 约 1-2 cm，max 约 3-5 cm。
- bag 中仍存在少数 3-5 秒级 final_odom 输出间隔，发生在上游 Loc 状态不可用、策略把所有源都 drop 的区间，不是人工 INS drop 窗口内的问题。

## 总结

10 秒 INS 人工丢失时，`loc_ekf_ins_imu_wheel_node` 的逻辑是正常的：INS 被 drop 后，wheel odom 和 IMU 会提升为 PRIMARY，并持续驱动 final_odom；INS 恢复后，final_odom 可以快速回到 INS 坐标。

需要注意的是，纯 IMU + wheel odom 的 10 秒 dead reckoning 漂移与路段强相关：多数窗口在 0.1 m 以内，个别窗口达到 1-3 m。该现象符合无绝对定位约束时的预期，后续若希望降低最坏窗口漂移，需要继续优化 wheel 尺度、IMU yaw 外参/零偏、以及 Loc 状态 LOST 时是否允许 wheel+IMU 兜底输出。
