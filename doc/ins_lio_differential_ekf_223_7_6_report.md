# ins_lio_differential_ekf 官方 EKF 调参测试报告

测试时间：2026-07-07

数据集：

```bash
cd ~/data/yangpu/lcp/0706/204-ok
rosbag play 223_7_6_* --clock
```

启动方式：

```bash
roslaunch robot_localization ins_lio_differential_ekf.launch set_use_sim_time:=true use_sim_time:=true
```

## 结论

原始 `ins_lio_differential_ekf.yaml` 不符合预期。它会让 `/localization/final_odom` 从 `(0, 0)` 附近开始，而输入的 `/localization/ins` 和 `/lio_loc_result` 第一帧都在真实 map 坐标 `(336.x, 27.x)` 附近，因此 final_odom 出现约 300 m 级别偏差。

根因不是“INS 主 + LIO differential 辅助”这个思路错误，而是 robot_localization 的初始化机制：

- EKF 第一帧 measurement 会初始化状态；
- LIO differential 的第一帧只用于保存 previous measurement，不会提供绝对 `x/y/yaw`；
- 如果 EKF 先被 differential/twist 类 measurement 初始化，`x/y/yaw` 会留在初始值 0；
- 默认 `initial_estimate_covariance` 很小，后续真实 INS 绝对 pose 与当前状态相差约 337 m，会被 Mahalanobis gate 拒绝；
- 之后滤波器只能在错误原点附近继续跑，所以出现百米级偏差。

当前采用方案：

- 保留官方 `ekf_localization_node`；
- 保留 INS 主源：`/localization/ins` 融合 `x/y/yaw + vx/vy/vyaw`；
- 保留 LIO 辅助：`/lio_loc_result` 使用 differential，只提供差分速度约束；
- 不使用额外 initializer 节点；
- 将 `initial_estimate_covariance` 放大，让 EKF 即使先被 differential/twist 初始化，也能在后续接受真实 INS absolute pose，而不是被 Mahalanobis gate 拒绝。

这个方案不写死初始坐标，也不需要 `ins_lio_ekf_initializer.py`。代价是：如果第一帧进入 EKF 的不是 INS absolute pose，仍可能短暂发布 `(0, 0)` 或产生一次从原点跳到真实 INS 坐标的跳变。测试中这种跳变只发生在启动阶段，后续轨迹可被 INS 拉回。

## 对比结果

| 配置 | final-LIO mean | final-LIO p95 | final-INS mean | final-INS p95 | yaw final-LIO mean | 主要现象 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 原始 INS absolute + LIO differential | 284.72 m | 337.30 m | 284.74 m | 337.29 m | 135.33 deg | final_odom 从原点起步，严重异常 |
| 原始配置 + 写死 initial_state 为第一帧 INS | 0.0440 m | 0.0962 m | 0.0018 m | 0.0105 m | 0.3582 deg | 正常，但初始坐标不能写死 |
| 原始配置 + 放大 initial covariance | 0.4741 m | 0.1010 m | 0.0017 m | 0.0107 m | 0.5272 deg | 能被 INS 拉回，不需要额外节点，但可能先发布原点并产生启动跳变 |
| INS absolute + LIO absolute 辅助 | 0.0390 m | 0.0837 m | 0.0019 m | 0.0109 m | 0.3666 deg | 正常，但不再是 LIO differential |
| INS set_pose 初始化 + INS 主 + LIO differential 辅助 | 0.0453 m | 0.1001 m | 0.0017 m | 0.0104 m | 0.3747 deg | 最干净，但需要额外 initializer 节点；当前不采用 |

## 仍需注意

官方 EKF 版本不会消费 `/localization/status` 和 `/localization/loc_policy_code`，所以它不能做到 `NORMAL + USE_FUSION_ODOM => Fusion PRIMARY, INS DROP` 这种动态策略；它只能依赖固定 topic 配置和协方差。

`-r 3 --duration=90` 快速测试中，最终方案的 `/clock` 曾从 `1783306778.575` 跳到 `1783306788.126`，导致 `/localization/final_odom` 也出现约 9.5 s 输出间隔和约 14.4 m 位移跳变。检查同一时间段 `/localization/ins` 和 `/lio_loc_result` 没有断档，因此这是快速回放 `/clock` 断档造成的测试现象，不是 EKF 权重或初始化逻辑导致。

官方 EKF 适合作为“INS 主、LIO differential 辅助”的官方对照链路；如果要按 Loc 状态码/策略码动态切换权重，仍应使用已有自定义策略节点。
