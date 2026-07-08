# robot_localization 初始化问题验证报告

测试数据：

```bash
cd ~/data/yangpu/lcp/0706/204-ok
rosbag play 223_7_6_* --clock --duration=12
```

## 现有配置检查

`params/ins_lio_differential_ekf.yaml` 当前关键配置：

- `odom0: /localization/ins`
- `odom0_config` 已打开 `x/y/yaw/vx/vy/vyaw`
- `odom0_differential: false`
- `odom0_relative: false`
- `odom1: /lio_loc_result`
- `odom1_config` 只打开 `x/y/yaw`
- `odom1_differential: true`
- `odom1_relative: false`

`launch/ins_lio_differential_ekf.launch` 当前加载：

- 官方 `ekf_localization_node`
- 节点名 `ins_lio_differential_ekf`
- rosparam 文件 `params/ins_lio_differential_ekf.yaml`
- 输出 remap 到 `/localization/final_odom`

因此 INS 本身没有被配置成 differential 或 relative，`x/y/yaw` 也确实打开了。

## 验证 1：INS-only

新增：

- `params/ins_only_init_check.yaml`
- `launch/ins_only_init_check.launch`

只融合 `/localization/ins`，且不设置 rejection threshold。

结果：

```text
first /localization/ins:
  t=1783306733.0189595
  x=336.661587188
  y=27.471616797

first /localization/final_odom_ins_only:
  t=1783306733.058936
  x=336.661587188
  y=27.471616797
```

结论：INS-only 时，官方 EKF 会从 INS 的真实 map 坐标初始化，不会从 `(0,0)` 起步。

## 验证 2：INS + LIO differential

新增：

- `params/ins_lio_init_check.yaml`
- `launch/ins_lio_init_check.launch`

融合 `/localization/ins` absolute 和 `/lio_loc_result` differential，且不设置 rejection threshold。

结果：

```text
first /lio_loc_result:
  t=1783306732.9781773
  x=336.644147859
  y=27.522372022

first /localization/ins:
  t=1783306733.0189595
  x=336.661587188
  y=27.471616797

first /localization/final_odom_ins_lio_check:
  t=1783306732.9881775
  x=0.0
  y=0.0

second /localization/final_odom_ins_lio_check:
  t=1783306733.0310104
  x=336.661587180
  y=27.471616803
```

结论：LIO differential 确实先于 INS 产生有效 measurement，并抢先初始化了 EKF。由于 differential pose 转成的是速度类测量，第一帧初始化时 `x/y/yaw` 保持默认 0。

在这个无 rejection threshold 的最小配置里，后续 INS absolute pose 能把状态拉回真实 map 坐标。

## 原始配置复现

使用当前 `ins_lio_differential_ekf.launch` 和 `ins_lio_differential_ekf.yaml` 短回放 12 秒。

结果：

```text
first /localization/final_odom:
  t=1783306733.09294
  x=0.0
  y=0.0

last /localization/final_odom after 12s:
  x=6.381135733
  y=0.087057710

final 到 INS 距离:
  mean=335.64 m
  max=337.78 m
```

原始配置中存在：

- `odom0_pose_rejection_threshold: 5.0`
- `odom1_pose_rejection_threshold: 5.0`
- 当前没有生效的 `initial_estimate_covariance` 大协方差

因此当 LIO differential 抢先把 EKF 初始化在 `(0,0)` 后，后续 INS absolute pose 与当前状态相差约 337 m，在 pose gate 下无法修正状态，final_odom 长时间停留在错误原点附近。

## 结论

1. INS-only 能从真实 map 坐标初始化。
2. INS+LIO differential 会出现第一帧 `(0,0)`，原因是 LIO differential 先于 INS 抢先初始化。
3. 原始配置之所以严重，是因为 LIO differential 抢先初始化后，INS absolute pose 被 Mahalanobis rejection threshold 挡住，无法把状态拉回真实 map 坐标。
4. 当前 yaml 的 INS 配置方向是对的：`odom0_differential=false`，`odom0_relative=false`，`odom0_config` 打开了 `x/y/yaw`。
5. 当前 launch 加载路径和节点命名空间是对的。

## 推荐修复

优先级：

1. 确保 `/localization/ins` 第一帧先进入 EKF，再启用 `/lio_loc_result` differential。
2. 使用官方 `/set_pose`，启动时用 `/localization/ins` 第一帧初始化滤波器。
3. 通过 launch 启动顺序或小 initializer 节点实现上述初始化门控。
4. 不建议把巨大 `initial_estimate_covariance` 作为主修复；它只能让后续 INS 有机会拉回状态，但无法避免启动时发布 `(0,0)` 或大跳变。
