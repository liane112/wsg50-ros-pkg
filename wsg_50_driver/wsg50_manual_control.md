# WSG-50 手动控制脚本使用说明

本文档对应：

```text
wsg50_manual_control.py
```

该脚本通过 ROS 位置话题手动控制 WSG-50 夹爪，支持固定增量开合、绝对宽度移动、速度设置、位置保持、完全张开和状态查看。

## 1. 当前控制方式

脚本使用位置控制：

```text
发布：/wsg_50_driver/goal_position
类型：wsg_50_common/Cmd
```

其中：

```text
pos   = 目标夹爪开口宽度，单位 mm
speed = 到达目标宽度时的移动速度，单位 mm/s
mode  = "move"
```

宽度越大表示夹爪越张开，宽度越小表示夹爪越闭合。

脚本不进行连续速度控制，也不订阅外部力传感器。每输入一条移动指令，只发布一次位置目标。

手动脚本工作在 ROS 接口层，不直接依赖底层 TCP 或 UDP。本说明按照当前实验中使用的 `UDP + auto_update` 方式启动 WSG 驱动。

## 2. 使用前注意事项

不要同时运行以下程序：

```text
wsg50_fsm_force_ctrl_v6.py
wsg50_manual_control.py
其他向 /wsg_50_driver/goal_position 发布消息的节点
```

手动脚本会检查位置话题上的其他发布者。检测到冲突后会发送最新实测宽度作为保持目标，并锁止后续移动命令。

当前版本不调用 `/stop`、`/homing` 或 `/ack` 服务。启动时会读取 `/wsg_50_driver/com_mode`，如果检测到模式不是 `auto_update`，脚本将拒绝启动。

```text
com_mode:=auto_update
```

## 3. 启动夹爪驱动

打开第一个 Ubuntu 终端：

```bash
cd ~/catkin_ws
source devel/setup.bash

roslaunch wsg_50_driver wsg_50_udp_script.launch \
  com_mode:=auto_update
```

其中：

```text
传输协议          UDP
WSG 远端端口      1500
本机 UDP 端口     1500
com_mode:=auto_update  使用自动状态更新模式
```

驱动连接成功后会自动执行一次 homing 和 tare。启动日志中应出现：

```text
Connecting to 192.168.1.20:1500 (udp); communication mode: auto_update ...
```

也可以在另一个终端检查当前协议和通信模式：

```bash
source ~/catkin_ws/devel/setup.bash
rosparam get /wsg_50_driver/protocol
rosparam get /wsg_50_driver/com_mode
```

预期依次输出：

```text
udp
auto_update
```

注意：当前 `wsg_50_udp_script.launch` 虽然声明了 `force` 启动参数，但 `grasping_force` 仍被写死为 `0.0`。因此，不要把 `force:=10` 写入当前 UDP 启动命令；即使传入也不会生效。

## 4. 启动手动控制脚本

打开第二个 Ubuntu 终端：

```bash
cd ~/catkin_ws
source devel/setup.bash

rosrun wsg_50_driver wsg50_manual_control.py \
  _min_width_mm:=1.0 \
  _max_width_mm:=109.0 \
  _min_speed_mm_s:=1.0 \
  _max_speed_mm_s:=50.0 \
  _speed_mm_s:=10.0 \
  _fine_step_mm:=0.2 \
  _normal_step_mm:=1.0 \
  _coarse_step_mm:=5.0 \
  _status_timeout_s:=0.5 \
  _hold_speed_mm_s:=1.0 \
  _exit_hold_wait_s:=0.1
```

准备完成后会显示当前状态和按键帮助。程序进入单键模式后，不再显示持续的 `wsg>` 提示符。

`o/p/i/c/v/x/s/t/h/q` 使用 ASCII 单键输入，按下后立即响应，不需要按 Enter。终端在等待按键期间关闭字符回显，程序退出或发生异常时会恢复原来的终端设置。

## 5. 手动指令

### 5.1 张开夹爪

```text
按 o    立即张开 1.0 mm
按 p    立即张开 5.0 mm
按 i    立即张开 0.2 mm
```

### 5.2 闭合夹爪

```text
按 c    立即闭合 1.0 mm
按 v    立即闭合 5.0 mm
按 x    立即闭合 0.2 mm
```

### 5.3 绝对位置

按一次：

```text
g
```

程序随即显示：

```text
absolute width (mm)>
```

输入 `60` 并按 Enter，夹爪将移动到 `60 mm` 的绝对开口宽度。

也可以按 `:` 进入完整命令模式，然后输入以下任意一条并按 Enter：

```text
g 60
goto 60
open
```

`open` 表示完全张开到配置的最大宽度。

### 5.4 速度设置

按一次：

```text
r
```

程序显示 `movement speed (mm/s)>` 后，输入 `20` 并按 Enter，表示将后续移动速度设为 `20 mm/s`。

也可以按 `:` 后输入：

```text
speed 20
```

速度必须位于：

```text
min_speed_mm_s <= speed <= max_speed_mm_s
```

速度设置只影响后续命令，不会立即使夹爪运动。

### 5.5 状态与维护

```text
按 s    发送最新实测宽度，保持当前位置
按 t    立即显示当前状态
按 h    显示帮助
按 ?    显示帮助
按 q    保持当前位置，等待位置消息发出后退出
按 :    进入完整命令模式
```

进入完整命令模式后，可以输入 `home`、`status`、`stop`、`help`、`open` 或 `q`，然后按 Enter 执行。

`home` 在 `auto_update` 模式下只会移动到配置的最大宽度，不会重新建立机械零点。驱动连接成功时已经自动执行过真正的 homing。`ack` 无法通过位置命令替代；输入 `ack` 只会报告该功能在 `auto_update` 下不可用。

状态输出示例：

```text
actual=59.000 mm target=59.000 mm speed=0.000 mm/s motor_force=0.000 N age=0.010 s state=IDLE
```

字段含义：

```text
actual       WSG 驱动反馈的实际开口宽度
target       手动脚本记录的最新目标宽度
speed        WSG 驱动反馈的实际速度
motor_force  WSG 驱动反馈的电机力，不是外部力传感器数据
age          最新状态消息的时间延迟
state        WSG 驱动报告的状态
```

## 6. 推荐操作顺序

首次操作建议使用较低速度，并先确认运动方向：

```text
按 r，输入 5 后按 Enter
按 t 查看状态
按 i
按 x
按 o
按 c
```

确认方向和状态正确后，再使用较大增量：

```text
按 p
按 v
```

需要移动到指定宽度时使用：

```text
按 g，输入 60 后按 Enter
```

实验结束时使用：

```text
按 q
```

脚本会先发布最新实测宽度作为保持目标，默认等待 `0.1 s`，然后退出。

## 7. 参数说明

| ROS 私有参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `~goal_position_topic` | `/wsg_50_driver/goal_position` | - | 位置命令话题 |
| `~status_topic` | `/wsg_50_driver/status` | - | WSG 状态话题 |
| `~driver_mode_param` | `/wsg_50_driver/com_mode` | - | 驱动通信模式参数 |
| `~min_width_mm` | `1.0` | mm | 软件允许的最小开口宽度 |
| `~max_width_mm` | `109.0` | mm | 软件允许的最大开口宽度 |
| `~min_speed_mm_s` | `1.0` | mm/s | 允许的最小移动速度 |
| `~max_speed_mm_s` | `50.0` | mm/s | 允许的最大移动速度 |
| `~speed_mm_s` | `10.0` | mm/s | 启动时的移动速度 |
| `~fine_step_mm` | `0.2` | mm | `i/x` 对应的增量 |
| `~normal_step_mm` | `1.0` | mm | `o/c` 对应的增量 |
| `~coarse_step_mm` | `5.0` | mm | `p/v` 对应的增量 |
| `~status_timeout_s` | `0.5` | s | 状态超时保持阈值 |
| `~startup_timeout_s` | `5.0` | s | 启动等待时间 |
| `~hold_speed_mm_s` | `1.0` | mm/s | 保持位置消息使用的速度 |
| `~exit_hold_wait_s` | `0.1` | s | 退出前等待保持消息发出的时间 |
| `~publisher_check_period_s` | `1.0` | s | 发布者冲突检查周期 |
| `~allow_shared_publisher` | `false` | - | 是否允许其他位置发布者 |

除非明确了解风险，不要设置：

```text
_allow_shared_publisher:=true
```

## 8. 控制频率

手动控制脚本没有固定的位置控制频率：

```text
按下一次增量键 -> 发布一次位置命令
```

相关频率如下：

```text
手动位置命令：由单键输入频率决定
安全状态检查：10 Hz
发布者冲突检查：默认 1 Hz
驱动 auto_update 状态读取配置：50 Hz
位置命令：收到 ROS 消息后立即进入驱动回调
```

`50 Hz` 是当前 launch 配置的状态读取频率，不代表夹爪能够以 `50 Hz` 完成机械位置变化。手动控制的实际命令频率由按键输入决定。

## 9. 软件保护

脚本包含以下保护：

1. 目标宽度必须位于软件范围内。
2. 移动速度必须位于软件范围内。
3. 必须收到时间有效的 WSG 状态。
4. WSG 驱动必须订阅位置命令话题。
5. 检测到其他位置命令发布者后发送保持目标并锁止移动。
6. 已发送运动命令后，如果状态超时则使用最后一次有效宽度发送保持目标。
7. 正常退出、EOF 或 `Ctrl+C` 时尝试发送保持目标。

该脚本不是安全等级控制器，不能代替硬件急停、设备限位和实验人员现场监护。

当前基础版本不使用外部传感器力作为停止条件。

当前 UDP launch 中实际配置为：

```text
grasping_force = 0.0
```

这表示驱动启动时不会执行 `setGraspingForceLimit()`。虽然 launch 文件声明了 `force` 参数，但该参数当前没有连接到 `grasping_force`，传入 `force:=10` 不会设置 10 N 的内部夹持力上限。该问题属于 UDP launch 配置，不影响手动位置指令的话题接口。

## 10. `auto_update` 保持逻辑

当前脚本只依赖：

```text
/wsg_50_driver/status
/wsg_50_driver/goal_position
```

按下 `s` 或 `q` 时，脚本读取最新的：

```text
/wsg_50_driver/status.width
```

然后发布：

```text
pos   = 最新实测宽度
speed = hold_speed_mm_s
mode  = "move"
```

WSG 驱动的 `auto_update` 位置回调收到消息后会先执行底层 `stop(true)`，再执行到该宽度的位置命令。因此，该方法可以用于普通手动实验中的停止和位置保持。

限制如下：

1. 保持位置来自最近一次状态反馈，存在一个反馈周期的延迟。
2. 状态已经超时时，只能使用最后一次已知宽度。
3. 如果驱动连接已经中断，保持消息无法到达夹爪。
4. 检测到另一个持续发布位置命令的节点时，单次保持消息无法保证压过对方命令。
5. 该逻辑不是安全等级急停，不能替代硬件急停。

## 11. 常见问题

### 11.1 启动后提示没有状态

检查驱动：

```bash
source ~/catkin_ws/devel/setup.bash
rostopic hz /wsg_50_driver/status
```

检查节点：

```bash
rosnode list
```

### 11.2 提示驱动没有订阅位置话题

检查：

```bash
rostopic info /wsg_50_driver/goal_position
```

应能看到 `/wsg_50_driver` 订阅该话题。

### 11.3 提示发布者冲突

检查：

```bash
rostopic info /wsg_50_driver/goal_position
```

关闭力控状态机或其他位置控制节点后，重新启动手动脚本。

### 11.4 提示驱动模式不是 `auto_update`

检查协议和通信模式：

```bash
rosparam get /wsg_50_driver/protocol
rosparam get /wsg_50_driver/com_mode
```

当前说明对应的输出应为：

```text
udp
auto_update
```

如果第二项不是 `auto_update`，停止手动脚本和驱动，然后使用：

```bash
roslaunch wsg_50_driver wsg_50_udp_script.launch \
  com_mode:=auto_update
```

### 11.5 指令被宽度范围限制

默认软件宽度范围是：

```text
[1.0, 109.0] mm
```

增量命令超过边界时，脚本会把目标限制在最近的边界。
