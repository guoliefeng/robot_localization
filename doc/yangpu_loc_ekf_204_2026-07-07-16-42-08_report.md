# 204_2026-07-07-16-42-08.bag 分析报告

分析日期：2026-07-07

Bag：

```text
/home/guoli/data/yangpu/lcp/0706/204-ok/test_out/204_2026-07-07-16-42-08.bag
```

## 结论

这次 `robot_localization / yangpu_loc_ekf` 的主逻辑是正常的：绝大部分时间处于：

```text
NORMAL + USE_FUSION_ODOM:
  INS = DROP
  Fusion = PRIMARY
```

因此 `/localization/final_odom` 主要跟随 `/lio_loc_result`，不是跟随 `/localization/ins`。

全包统计：

| 对比项 | mean | p95 | max |
| --- | ---: | ---: | ---: |
| final 到 INS 平面距离 | 0.111 m | 0.257 m | 0.354 m |
| final 到 Fusion 平面距离 | 0.0076 m | 0.0239 m | 0.219 m |
| final 到 INS yaw 差 | 0.247 deg | 0.940 deg | 1.179 deg |
| final 到 Fusion yaw 差 | 0.0020 deg | 0.0033 deg | 0.591 deg |

5372 个 final 样本中：

```text
更靠近 Fusion：5268
更靠近 INS：104
```

## Bag 基本信息

时间范围：

```text
start: 1783306732.978177
end:   1783306938.771635
duration: 205.793 s
```

主要 topic：

| topic | 数量 |
| --- | ---: |
| `/localization/final_odom` | 5372 |
| `/localization/ins` | 18752 |
| `/lio_loc_result` | 20577 |
| `/localization/status` | 2742 |
| `/localization/loc_policy_code` | 2743 |

说明：bag 中仍记录了 `/localization/loc_status` 和 `/chcnav/devpvt`，但当前 `yangpu_loc_ekf_node` 不订阅它们。robot_localization 只消费：

```text
/localization/ins
/lio_loc_result
/localization/status
/localization/loc_policy_code
```

## 状态与策略时间分布

按 `/localization/status`：

| status | 持续时间 |
| --- | ---: |
| NORMAL | 198.052 s |
| LOST | 6.281 s |
| NOT_STABLE | 1.243 s |
| SECONDARY | 0.212 s |

按 `/localization/loc_policy_code`：

| policy | 含义 | 持续时间 |
| --- | --- | ---: |
| 2 | USE_FUSION_ODOM | 199.295 s |
| 36 | USE_GNSS + USE_RELOC_GNSS | 0.212 s |
| 64 | USE_RELOC_X | 6.281 s |

按 `yangpu_loc_ekf` 推导出的输入模式：

| INS | Fusion | 持续时间 | 说明 |
| --- | --- | ---: | --- |
| DROP | PRIMARY | 198.052 s | NORMAL + USE_FUSION_ODOM，主工作模式 |
| DROP | DROP | 6.281 s | LOST + USE_RELOC_X |
| WEAK | DEGRADED | 1.243 s | NOT_STABLE + USE_FUSION_ODOM |
| PRIMARY | WEAK | 0.212 s | SECONDARY + USE_GNSS/RELOC_GNSS |

## 权重如何调节

当前 `yangpu_loc_ekf_node` 使用 `/localization/status` 和 `/localization/loc_policy_code` 选择输入模式。

### NORMAL + USE_FUSION_ODOM

```text
INS = DROP
Fusion = PRIMARY
```

robot_localization 动作：

- INS 回调会收到 `/localization/ins`，但直接 drop，不送入 EKF。
- Fusion 回调会收到 `/lio_loc_result`，作为 PRIMARY 送入 EKF。
- Fusion covariance：

```text
pose x/y: 0.05
yaw:      (1 deg)^2
twist vx/vy: 0.20
twist vyaw:  (1 deg)^2
```

该模式下统计：

| 对比项 | mean | p95 | max |
| --- | ---: | ---: | ---: |
| final 到 INS | 0.112 m | 0.257 m | 0.354 m |
| final 到 Fusion | 0.0069 m | 0.0215 m | 0.105 m |
| final yaw 到 INS | 0.248 deg | 0.943 deg | 1.179 deg |
| final yaw 到 Fusion | 0.0007 deg | 0.0026 deg | 0.091 deg |

结论：final 明显贴 Fusion。

### NOT_STABLE + USE_FUSION_ODOM

```text
INS = WEAK
Fusion = DEGRADED
```

robot_localization 动作：

- Fusion 仍送入 EKF，但 covariance 放大。
- INS 作为弱约束送入 EKF。

该模式持续约 1.243 s，统计：

| 对比项 | mean | p95 | max |
| --- | ---: | ---: | ---: |
| final 到 INS | 0.080 m | 0.201 m | 0.235 m |
| final 到 Fusion | 0.066 m | 0.188 m | 0.219 m |

结论：NOT_STABLE 期间 final 处于过渡状态，Fusion 权重降低，INS 以弱约束参与，因此 final 介于两者附近。

### SECONDARY + USE_GNSS/RELOC_GNSS

```text
INS = PRIMARY
Fusion = WEAK
```

该模式只持续约 0.212 s。统计：

| 对比项 | mean | p95 | max |
| --- | ---: | ---: | ---: |
| final 到 INS | 0.0035 m | 0.0091 m | 0.0096 m |
| final 到 Fusion | 0.0381 m | 0.0536 m | 0.0537 m |

结论：这 0.2 秒内 final 会短暂跟 INS。

### LOST + USE_RELOC_X

```text
INS = DROP
Fusion = DROP
```

robot_localization 动作：

- INS 和 Fusion 都不作为 measurement 送入 EKF。
- EKF 只能依赖内部预测/上一状态。
- 因此可能出现 final_odom 暂停发布或短时外推。

该模式总计约 6.281 s。

## 关键时间线

相对 bag 起点的主要片段：

| 时间段 | 持续 | status | policy | INS | Fusion |
| --- | ---: | --- | --- | --- | --- |
| 0.149-0.260 s | 0.111 s | SECONDARY | USE_GNSS+USE_RELOC_GNSS | PRIMARY | WEAK |
| 0.260-1.916 s | 1.656 s | NORMAL | USE_FUSION_ODOM | DROP | PRIMARY |
| 1.916-4.964 s | 3.048 s | LOST | USE_RELOC_X | DROP | DROP |
| 5.338-5.967 s | 0.629 s | NOT_STABLE | USE_FUSION_ODOM | WEAK | DEGRADED |
| 5.967-37.004 s | 31.037 s | NORMAL | USE_FUSION_ODOM | DROP | PRIMARY |
| 37.004-39.890 s | 2.887 s | LOST | USE_RELOC_X | DROP | DROP |
| 40.385-40.867 s | 0.483 s | NOT_STABLE | USE_FUSION_ODOM | WEAK | DEGRADED |
| 40.867-107.740 s | 66.873 s | NORMAL | USE_FUSION_ODOM | DROP | PRIMARY |
| 107.740-107.883 s | 0.143 s | LOST | USE_RELOC_X | DROP | DROP |
| 107.883-141.922 s | 34.039 s | NORMAL | USE_FUSION_ODOM | DROP | PRIMARY |
| 141.922-142.053 s | 0.131 s | NOT_STABLE | USE_FUSION_ODOM | WEAK | DEGRADED |
| 142.053-176.415 s | 34.362 s | NORMAL | USE_FUSION_ODOM | DROP | PRIMARY |
| 176.476-205.793 s | 29.318 s | NORMAL | USE_FUSION_ODOM | DROP | PRIMARY |

## 切换时 final_odom 的表现

典型切换：

| 相对时间 | 切换 | 切换前 final 到 INS/Fusion mean | 切换后 final 到 INS/Fusion mean |
| ---: | --- | --- | --- |
| 0.260 s | SECONDARY/GNSS -> NORMAL/FUSION | INS 0.000 m / Fusion 0.053 m | INS 0.044 m / Fusion 0.007 m |
| 5.338 s | NORMAL/FUSION -> NOT_STABLE/FUSION | INS 0.026 m / Fusion 0.008 m | INS 0.031 m / Fusion 0.022 m |
| 5.967 s | NOT_STABLE/FUSION -> NORMAL/FUSION | INS 0.029 m / Fusion 0.032 m | INS 0.030 m / Fusion 0.006 m |
| 40.867 s | NOT_STABLE/FUSION -> NORMAL/FUSION | INS 0.074 m / Fusion 0.071 m | INS 0.029 m / Fusion 0.011 m |

解释：

- 切到 `NORMAL + USE_FUSION_ODOM` 后，final 会快速贴近 Fusion。
- 切到 `NOT_STABLE + USE_FUSION_ODOM` 时，Fusion 降为 DEGRADED，INS 变为 WEAK，final 会稍微离 Fusion 远一点。
- `SECONDARY + USE_GNSS` 极短，此时 final 会临时贴近 INS。

## final_odom 输出间隔

整体：

```text
final_odom 平均记录频率: 26.125 Hz
gap p95: 0.050 s
gap max: 2.680 s
step p95: 0.081 m
step max: 3.616 m
```

大于 0.2 s 的 final_odom gap：

| 相对时间 | gap | 状态/策略 | 输入是否连续 | final step |
| --- | ---: | --- | --- | ---: |
| 1.701-4.381 s | 2.680 s | LOST / USE_RELOC_X | INS/Fusion 连续 | 0.039 m |
| 36.771-39.001 s | 2.230 s | LOST / USE_RELOC_X | INS/Fusion 连续 | 2.243 m |
| 71.931-72.931 s | 1.000 s | NORMAL / USE_FUSION_ODOM | INS/Fusion 连续 | 1.529 m |
| 105.151-107.481 s | 2.330 s | NORMAL / USE_FUSION_ODOM | INS/Fusion 连续 | 3.616 m |
| 139.571-141.661 s | 2.090 s | NORMAL / USE_FUSION_ODOM | INS/Fusion 连续 | 0.000 m |
| 173.831-176.131 s | 2.300 s | NORMAL / USE_FUSION_ODOM | INS/Fusion 连续 | 3.532 m |

注意：这些 gap 期间 `/localization/ins` 和 `/lio_loc_result` 本身是连续的，Fusion 最大间隔约 0.01-0.015 s。因此 gap 不是上游 odom 没数据造成的，更像是 robot_localization 发布/处理侧停顿，或 rosbag record/play 压力导致的 final_odom 记录缺口。

## 本次发生了什么

1. Loc 大部分时间输出 `status=NORMAL`，`policy=USE_FUSION_ODOM`。
2. `yangpu_loc_ekf_node` 按策略把 INS drop，只把 Fusion 作为 PRIMARY measurement 送入 EKF。
3. 在极短的 `NOT_STABLE` 区间，Fusion 降权为 DEGRADED，INS 作为 WEAK 参与。
4. 在极短的 `SECONDARY + GNSS/RELOC_GNSS` 区间，INS 成为 PRIMARY，Fusion 为 WEAK。
5. 在 `LOST + USE_RELOC_X` 区间，INS/Fusion 都 drop，EKF 没有新的 odom measurement。
6. final_odom 整体明显贴 Fusion，符合 `NORMAL + USE_FUSION_ODOM => Fusion PRIMARY, INS DROP` 的预期。

## 建议

1. 当前权重策略是正确的，不需要再把 INS 加回 `NORMAL + USE_FUSION_ODOM`。
2. 后续重点应排查 final_odom 的 1-2 秒发布 gap，尤其是发生在 `NORMAL + USE_FUSION_ODOM` 且 Fusion 输入连续的几个时间点。
3. 如果要继续定位 gap 原因，建议下一次 roslaunch 时保存 `yangpu_loc_ekf` 的 screen log，重点查是否有 update rate warning、Mahalanobis rejection、时间戳回退或 filter predict/update 停顿。
