# WSG50 力控状态机 v5：1N 稳定预加载后再启动推理

日期：2026-07-03

本文档对应新脚本：

```text
wsg50_fsm_force_ctrl_v5.py
```

v5 的目标是：

```text
先稳定接触并把接触力维持在 1.0 N 附近；
然后发布 policy_enable=True 通知策略开始推理；
等策略在 enable 之后发布新的目标力；
最后再进入 FORCE，用网络目标力平滑接管。
```

---

## 1. 版本和启动命令的对应关系

| 版本/模式 | 脚本 | 关键参数 | 状态链路 | 用途 |
| --- | --- | --- | --- | --- |
| v3 直接力控 | `wsg50_fsm_force_ctrl.py` | `_contact_pipeline_enable:=false` | `APPROACH -> FORCE` | 对照旧流程，接触后直接用目标力 |
| v4 接触保护 | `wsg50_fsm_force_ctrl.py` | `_contact_pipeline_enable:=true` | `APPROACH -> CONTACT_CAPTURE -> PRELOAD -> FORCE` | 接触捕获、预加载、过冲恢复、慢接管 |
| v5 策略后启动 | `wsg50_fsm_force_ctrl_v5.py` | `_contact_pipeline_enable:=true`，`_policy_wait_after_preload_enable:=true` | `APPROACH -> CONTACT_CAPTURE -> PRELOAD -> WAIT_POLICY_TARGET -> FORCE` | 先稳定 1N，再启动推理，再使用新目标力 |
| v5 兼容 v4 | `wsg50_fsm_force_ctrl_v5.py` | `_policy_wait_after_preload_enable:=false` | `APPROACH -> CONTACT_CAPTURE -> PRELOAD -> FORCE` | 使用 v5 脚本，但不等待策略使能后的新目标 |

v5 新增状态：

```text
WAIT_POLICY_TARGET
```

它做三件事：

```text
1. 继续用 PRELOAD 控制把接触力维持在 preload_target_N。
2. 发布 policy_enable=True。
3. 等待 /znsv6_cmd/act2 在 policy_enable 之后产生新消息。
```

---

## 2. v5 新增参数

1. 策略等待总开关
   - 参数：`policy_wait_after_preload_enable`
   - v5 默认值：`True`
   - 物理意义：PRELOAD 稳定后不直接进入 FORCE，而是先进入 `WAIT_POLICY_TARGET`。

2. 策略使能话题
   - 参数：`policy_enable_topic`
   - v5 默认值：`~policy_enable`
   - 默认完整话题：`/wsg50_fsm_force_ctrl/policy_enable`
   - 消息类型：`std_msgs/Bool`
   - 物理意义：`False` 表示策略不应开始推理或不应发布有效目标；
     `True` 表示预加载已稳定，策略可以开始推理。

3. 是否要求目标力必须晚于使能时刻
   - 参数：`policy_require_target_after_enable`
   - v5 默认值：`True`
   - 物理意义：防止使用 PRELOAD 之前缓存的旧目标力。

4. 等待策略目标超时
   - 参数：`policy_wait_timeout_s`
   - v5 默认值：`5.0 s`
   - 物理意义：进入 `WAIT_POLICY_TARGET` 后，超过该时间还没有新目标力，
     就进入 `OPEN_TO_START`，原因是 `policy_wait_timeout`。

5. 策略目标确认时间
   - 参数：`policy_ready_confirm_s`
   - v5 默认值：`0.10 s`
   - 物理意义：使能后的新目标力需要短时间保持新鲜，避免单帧误触发。

6. 使能时是否清空目标力滤波
   - 参数：`policy_reset_target_filter_on_enable`
   - v5 默认值：`True`
   - 物理意义：清除策略使能前残留的目标力低通值；
     等使能后的新目标力到来后重新建立滤波。

---

## 3. 策略节点需要配合的行为

推荐策略节点订阅：

```text
/wsg50_fsm_force_ctrl/policy_enable
```

推荐语义：

```text
policy_enable=False：
  不开始推理，或不发布有效目标力。

policy_enable=True：
  开始推理，并发布 /znsv6_cmd/act2。
```

如果策略节点暂时不能停止发布，也可以继续发布。
v5 仍会要求目标力消息时间戳晚于 `policy_enable_time_s`，
因此会等 enable 后的新一帧目标力再进入 FORCE。
但更推荐从策略端真正按 `policy_enable` 控制推理启动，
这样图像和触觉输入对应的是 1N 稳定接触后的状态。

---

## 4. 推荐启动命令：v5 完整模式

功能：

```text
开 v5 状态机；
开接触捕获；
开 backoff；
开过冲恢复；
先稳定到 1.0 N；
再发布 policy_enable=True；
再等待 enable 后的新目标力；
进入 FORCE 后慢速接管网络目标。
```

```bash
cd ~/catkin_ws
source devel/setup.bash

rosrun wsg_50_driver wsg50_fsm_force_ctrl_v5.py \
  _goal_position_topic:=/wsg_50_driver/goal_position \
  _status_topic:=/wsg_50_driver/status \
  _measured_force_topic:=/znsv6_data_sensor2 \
  _measured_force_index:=0 \
  _target_force_topic:=/znsv6_cmd/act2 \
  _target_force_index:=0 \
  _debug_topic:=/debug \
  _target_ctrl_topic:=/wsg50_fsm_force_ctrl/target_ctrl \
  _policy_enable_topic:=/wsg50_fsm_force_ctrl/policy_enable \
  _contact_pipeline_enable:=true \
  _force_threshold_N:=0.15 \
  _capture_backoff_enable:=true \
  _capture_backoff_mm:=0.10 \
  _capture_backoff_speed_mm_s:=4.0 \
  _overshoot_recovery_enable:=true \
  _capture_normal_high_N:=1.8 \
  _overshoot_timeout_s:=3.0 \
  _overshoot_k_open_mm_per_N:=0.06 \
  _overshoot_max_step_mm:=0.15 \
  _overshoot_open_speed_mm_s:=6.0 \
  _preload_target_N:=1.0 \
  _preload_low_N:=0.90 \
  _preload_high_N:=1.10 \
  _preload_timeout_s:=20.0 \
  _preload_min_hold_s:=1.0 \
  _preload_ready_confirm_s:=1.5 \
  _preload_kp_mm_per_N:=0.12 \
  _preload_speed_mm_s:=3.0 \
  _preload_dforce_max_N_per_s:=0.3 \
  _policy_wait_after_preload_enable:=true \
  _policy_wait_timeout_s:=5.0 \
  _policy_ready_confirm_s:=0.10 \
  _policy_require_target_after_enable:=true \
  _policy_reset_target_filter_on_enable:=true \
  _policy_blend_s:=4.0 \
  _target_rise_rate_N_per_s:=0.2 \
  _target_fall_rate_N_per_s:=2.0 \
  _target_force_lpf_alpha:=0.1 \
  _target_scale:=1.0 \
  _pid_speed_mm_s:=3.0 \
  _kp_mm_per_N:=0.08 \
  _ki_mm_per_Ns:=0.0 \
  _kd_mm_per_Ns:=0.0 \
  _pos_eps_mm:=0.005 \
  _cmd_min_period_s:=0.015 \
  _min_speed_mm_s:=1.0 \
  _debug_period_s:=0.05
```

---

## 5. v5 只有过冲保护，不做自动松开判断

功能：

```text
仍然先 1N 稳定接触，再启动策略；
仍然有 CONTACT_CAPTURE / OVERSHOOT_RECOVERY；
但是 FORCE 中不因为目标力下降趋势或失接触自动 OPEN_TO_START。
```

只需要在完整 v5 命令后追加：

```bash
  _trend_open_enable:=false \
  _force_contact_lost_to_open_enable:=false
```

完整命令：

```bash
cd ~/catkin_ws
source devel/setup.bash

rosrun wsg_50_driver wsg50_fsm_force_ctrl_v5.py \
  _goal_position_topic:=/wsg_50_driver/goal_position \
  _status_topic:=/wsg_50_driver/status \
  _measured_force_topic:=/znsv6_data_sensor2 \
  _measured_force_index:=0 \
  _target_force_topic:=/znsv6_cmd/act2 \
  _target_force_index:=0 \
  _debug_topic:=/debug \
  _target_ctrl_topic:=/wsg50_fsm_force_ctrl/target_ctrl \
  _policy_enable_topic:=/wsg50_fsm_force_ctrl/policy_enable \
  _contact_pipeline_enable:=true \
  _force_threshold_N:=0.15 \
  _capture_backoff_enable:=true \
  _capture_backoff_mm:=0.10 \
  _capture_backoff_speed_mm_s:=4.0 \
  _overshoot_recovery_enable:=true \
  _capture_normal_high_N:=1.8 \
  _overshoot_timeout_s:=3.0 \
  _overshoot_k_open_mm_per_N:=0.06 \
  _overshoot_max_step_mm:=0.15 \
  _overshoot_open_speed_mm_s:=6.0 \
  _preload_target_N:=1.0 \
  _preload_low_N:=0.90 \
  _preload_high_N:=1.10 \
  _preload_timeout_s:=20.0 \
  _preload_min_hold_s:=1.0 \
  _preload_ready_confirm_s:=1.5 \
  _preload_kp_mm_per_N:=0.12 \
  _preload_speed_mm_s:=3.0 \
  _preload_dforce_max_N_per_s:=0.3 \
  _policy_wait_after_preload_enable:=true \
  _policy_wait_timeout_s:=5.0 \
  _policy_ready_confirm_s:=0.10 \
  _policy_require_target_after_enable:=true \
  _policy_reset_target_filter_on_enable:=true \
  _policy_blend_s:=4.0 \
  _target_rise_rate_N_per_s:=0.2 \
  _target_fall_rate_N_per_s:=2.0 \
  _target_force_lpf_alpha:=0.1 \
  _target_scale:=1.0 \
  _pid_speed_mm_s:=3.0 \
  _kp_mm_per_N:=0.08 \
  _ki_mm_per_Ns:=0.0 \
  _kd_mm_per_Ns:=0.0 \
  _pos_eps_mm:=0.005 \
  _cmd_min_period_s:=0.015 \
  _min_speed_mm_s:=1.0 \
  _trend_open_enable:=false \
  _force_contact_lost_to_open_enable:=false \
  _debug_period_s:=0.05
```

---

## 6. v5 兼容 v4 行为

如果要使用 v5 脚本，但不等待策略使能后的新目标力：

```bash
_policy_wait_after_preload_enable:=false
```

此时链路变为：

```text
APPROACH -> CONTACT_CAPTURE -> PRELOAD -> FORCE
```

注意：v5 脚本的默认预加载目标是 `1.0 N`，
而 v4 脚本当前默认预加载目标是 `0.7 N`。
如果想更接近 v4 当前默认值，需要显式设置：

```bash
_preload_target_N:=0.7 \
_preload_low_N:=0.5 \
_preload_high_N:=0.9
```

---

## 7. 小位移过滤相关参数

当前脚本有两个会过滤小命令的参数：

```text
pos_eps_mm
cmd_min_period_s
```

1. `pos_eps_mm`
   - 默认值：`0.03 mm`
   - 含义：如果新命令宽度和上次发送宽度差小于该值，
     且速度相同，则不重复发送。
   - v5 推荐实验值：`0.005 mm`
   - 原因：1N 预加载附近误差很小时，位移修正可能只有 `0.01 mm` 量级；
     默认 `0.03 mm` 容易把这些小修正过滤掉。

2. `cmd_min_period_s`
   - 默认值：约 `0.0267 s`
   - 含义：两次命令间隔太短时不发送。
   - v5 推荐实验值：`0.015 s`

3. `min_speed_mm_s`
   - 默认值：`5.0 mm/s`
   - 含义：所有速度命令会被下限夹到至少 `5.0 mm/s`。
   - v5 推荐实验值：`1.0 mm/s`
   - 原因：如果命令里写 `_preload_speed_mm_s:=3.0`，
     但不降低 `min_speed_mm_s`，实际发送速度仍会变成 `5.0 mm/s`。

推荐组合：

```bash
_pos_eps_mm:=0.005 \
_cmd_min_period_s:=0.015 \
_min_speed_mm_s:=1.0
```

---

## 8. 配套可视化

三条曲线：

```text
target raw   -> /znsv6_cmd/act2
actual raw   -> /znsv6_data_sensor2
target_ctrl  -> /wsg50_fsm_force_ctrl/target_ctrl
```

v5 中 `target_ctrl` 的含义：

```text
PRELOAD / WAIT_POLICY_TARGET：发布 preload_target_N，也就是 1.0 N 平台。
FORCE：发布实际进入 PID 的目标力，也就是 blend/rate limit 后的 target_ctrl。
```

启动命令：

```bash
cd ~/catkin_ws
source devel/setup.bash

rosrun sensor_recorder pyqtgraph_live_viewer.py \
  _image_rotate_deg:=-90 \
  --image-topic /cam_4/color/image_raw \
  --target-topic /znsv6_cmd/act2 \
  --actual-topic /znsv6_data_sensor2 \
  --control-topic /wsg50_fsm_force_ctrl/target_ctrl
```

debug 话题：

```bash
rostopic echo /debug
```

v5 重点字段：

```text
state=WAIT_POLICY_TARGET
policy_enabled=True
policy_target_ready=True/False
policy_wait_elapsed=...
target_ctrl_blend=...
preload_ready=True
```
