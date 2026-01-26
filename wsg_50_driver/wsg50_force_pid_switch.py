#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WSG-50 力控模式切换（状态机版，不依赖实时status更新）
- 使用最后有效的状态数据，即使/status话题停止更新也能继续运行
- INIT    : 上线即"首发唤醒" -> 回到 start_width（force_send=True）
- AWAKE   : 等第一帧 /status；超时 awake_timeout_s 秒则强制进入 APPROACH
- APPROACH: 未接触靠近（110→0），近接自动降速/减步；接触上阈 -> FORCE
- FORCE   : 目标力与实测力闭环 PID，输出位置增量（带步长/速度限幅）；力低于下阈 -> 回 APPROACH
- 发布 /wsg50/contact: std_msgs/Bool
- 与 wsg_50_driver 对接：发布 wsg_50_common/Cmd(goal_position)，订阅 wsg_50_common/Status(status)
"""

import math
import rospy
from enum import Enum
from rospy import AnyMsg
from roslib.message import get_message_class
from std_msgs.msg import Bool
from wsg_50_common.msg import Cmd, Status

def clamp(x, lo, hi): 
    return max(lo, min(hi, x))

def extract_scalar_from_msg(msg, index=0):
    """从各种消息类型中提取标量值，支持多种消息格式"""
    # 1. 处理带data数组的消息
    if hasattr(msg, "data") and hasattr(msg.data, "data") and isinstance(msg.data.data, (list, tuple)):
        arr = msg.data.data
        if len(arr) > index:
            return float(arr[index])
    
    # 2. 处理标准Float32MultiArray类型
    if hasattr(msg, "data") and isinstance(msg.data, (list, tuple)):
        if len(msg.data) > index:
            return float(msg.data[index])
    
    # 3. 处理单个浮点数
    if hasattr(msg, "data") and isinstance(msg.data, (float, int)):
        return float(msg.data)
    
    rospy.logwarn_throttle(1.0, "无法提取数据，消息类型: %s", type(msg).__name__)
    return None

class Phase(Enum):
    """状态机阶段定义"""
    INIT=0; AWAKE=1; APPROACH=2; FORCE=3

class WSG50ForcePIDSwitchSM(object):
    def __init__(self):
        rospy.init_node("wsg50_force_pid_switch")

        # 话题 / 参数
        self.goal_position_topic = rospy.get_param("~goal_position_topic", "/wsg_50_driver/goal_position")
        self.status_topic        = rospy.get_param("~status_topic",        "/wsg_50_driver/status")

        self.target_force_topic   = rospy.get_param("~target_force_topic",   "/znsv6_control")
        self.measured_force_topic = rospy.get_param("~measured_force_topic", "/znsv6_data_sensor1")
        self.target_index   = int(rospy.get_param("~target_force_index", 0))
        self.measured_index = int(rospy.get_param("~measured_force_index", 0))
        self.target_scale   = float(rospy.get_param("~target_force_scale", 1.0))
        self.measured_scale = float(rospy.get_param("~measured_force_scale", 1.0))
        self.measured_abs   = bool(rospy.get_param("~measured_force_abs", True))

        # 接触迟滞
        self.contact_on_threshold  = float(rospy.get_param("~contact_on_threshold_N", 2.0))
        self.contact_off_threshold = float(rospy.get_param("~contact_off_threshold_N", 1.2))
        self.contact_topic = rospy.get_param("~contact_topic", "/wsg50/contact")

        # 位置/速度限幅
        self.min_width = float(rospy.get_param("~min_width_mm", 0.0))
        self.max_width = float(rospy.get_param("~max_width_mm", 110.0))
        self.min_speed = float(rospy.get_param("~min_speed_mm_s", 5.0))
        self.max_speed = float(rospy.get_param("~max_speed_mm_s", 420.0))

        # 未接触靠近
        self.approach_speed = float(rospy.get_param("~approach_speed_mm_s", 20.0))
        self.approach_step  = float(rospy.get_param("~approach_step_mm",     0.5))
        self.approach_cmd_period = float(rospy.get_param("~approach_cmd_period_s", 0.05))
        self.start_width    = float(rospy.get_param("~start_width_mm",      110.0))  # 默认 110

        # 近接减速/减步
        self.near_contact_ratio   = float(rospy.get_param("~near_contact_ratio", 0.7))
        self.approach_step_near   = float(rospy.get_param("~approach_step_near_mm", 0.2))
        self.approach_speed_near  = float(rospy.get_param("~approach_speed_near_mm_s", 10.0))

        # 力环 PID（mm/N）
        self.kp = float(rospy.get_param("~kp_mm_per_N",   0.15))
        self.ki = float(rospy.get_param("~ki_mm_per_Ns",  0.00))
        self.kd = float(rospy.get_param("~kd_mm_per_Ns",  0.00))
        self.pid_speed = float(rospy.get_param("~pid_speed_mm_s", 40.0))
        self.deadband  = float(rospy.get_param("~deadband_N", 0.2))
        self.i_limit   = float(rospy.get_param("~i_limit_mm", 5.0))
        self.pid_step_limit_mm = float(rospy.get_param("~pid_step_limit_mm", 0.5))

        # 力低通
        self.force_alpha = float(rospy.get_param("~force_lpf_alpha", 0.3))

        # 控制循环频率
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))

        # 命令去抖：位置死区 + 最小发送周期
        self.pos_eps_mm        = float(rospy.get_param("~pos_eps_mm", 0.05))
        self.cmd_min_period_s  = float(rospy.get_param("~cmd_min_period_s", 1.0/self.rate_hz*0.8))

        # —— 新增：AWAKE 超时 & /status 追踪 ——
        self.awake_timeout_s = float(rospy.get_param("~awake_timeout_s", 0.8))
        self._status_rx = False                    # 是否收到过至少一帧 /status
        self.status_last_rx_t = rospy.Time(0)      # 最近一帧 /status 的时间
        self.phase_enter_t = rospy.Time.now()      # 当前状态进入时刻

        # 运行时变量
        self.phase = Phase.INIT
        self._has_any_cmd_sent = False

        # 使用最后有效的状态数据
        self.current_width = None
        self.target_force  = None
        self.meas_force_f  = None

        # PID 控制变量
        self.int_acc = 0.0
        self.prev_err = None
        self.prev_t   = rospy.Time.now()

        # 时间追踪
        self.last_approach_cmd_t = rospy.Time(0)

        # 命令历史
        self._last_cmd_pos = None
        self._last_cmd_spd = None
        self._last_cmd_t   = rospy.Time(0)

        # 力话题实际类型
        self._target_msg_cls = None
        self._meas_msg_cls   = None

        # 通信
        self.cmd_pub     = rospy.Publisher(self.goal_position_topic, Cmd,  queue_size=10)
        self.contact_pub = rospy.Publisher(self.contact_topic,       Bool, queue_size=10)
        rospy.Subscriber(self.status_topic,        Status, self._status_cb, queue_size=50)
        rospy.Subscriber(self.target_force_topic,  AnyMsg, self._target_cb_any,   queue_size=50)
        rospy.Subscriber(self.measured_force_topic,AnyMsg, self._measured_cb_any, queue_size=100)
        rospy.loginfo("WSG-50 Force-PID switch (SM) node started.")

    # 回调 - 状态更新
    def _status_cb(self, msg: Status):
        """更新夹爪状态（宽度）"""
        self.current_width = float(msg.width)   # mm
        self._status_rx = True
        self.status_last_rx_t = rospy.Time.now()
        rospy.logdebug("Status updated: width=%.2f mm", self.current_width)

    # 回调 - 目标力更新
    def _target_cb_any(self, any_msg: AnyMsg):
        """更新目标力值"""
        try:
            # 首次接收时确定消息类型
            if self._target_msg_cls is None:
                typ = any_msg._connection_header.get('type','')
                self._target_msg_cls = get_message_class(typ)
                rospy.loginfo("Detected target_force msg type: %s", typ)
            
            # 解析消息
            m = self._target_msg_cls()
            m.deserialize(any_msg._buff)
            v = extract_scalar_from_msg(m, self.target_index)
            if v is not None: 
                self.target_force = float(v) * self.target_scale
                rospy.logdebug("Target force updated: %.2f N", self.target_force)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Target force parse failed: %s", e)

    # 回调 - 测量力更新
    def _measured_cb_any(self, any_msg: AnyMsg):
        """更新测量力值（带低通滤波）"""
        try:
            # 首次接收时确定消息类型
            if self._meas_msg_cls is None:
                typ = any_msg._connection_header.get('type','')
                self._meas_msg_cls = get_message_class(typ)
                rospy.loginfo("Detected measured_force msg type: %s", typ)
            
            # 解析消息
            m = self._meas_msg_cls()
            m.deserialize(any_msg._buff)
            v = extract_scalar_from_msg(m, self.measured_index)
            if v is None: 
                rospy.logwarn_throttle(1.0, "无法提取测量力值")
                return
            
            # 处理绝对值
            f = float(v) * self.measured_scale
            if self.measured_abs: 
                f = abs(f)
            
            # 低通滤波更新
            if self.meas_force_f is None:
                self.meas_force_f = f
            else:
                alpha = clamp(self.force_alpha, 0.0, 1.0)
                self.meas_force_f = alpha * f + (1.0 - alpha) * self.meas_force_f
            
            rospy.logdebug("Measured force updated: %.2f N (filtered: %.2f N)", f, self.meas_force_f)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Measured force parse failed: %s", e)

    # 发布接触状态
    def _publish_contact(self, flag: bool):
        """发布接触状态"""
        self.contact_pub.publish(Bool(data=bool(flag)))
        rospy.loginfo("Contact state changed: %s", "ON" if flag else "OFF")

    # 统一命令发送（首发豁免 + 去抖 + 周期限制）
    def _send_goal(self, width_mm, speed_mm_s, force_send=False):
        """发送位置命令（带去抖和限幅）"""
        now = rospy.Time.now()
        
        # 限幅处理
        width_mm = clamp(width_mm, self.min_width, self.max_width)
        speed_mm_s = clamp(abs(speed_mm_s), self.min_speed, self.max_speed)
        
        # 首发命令豁免
        if not self._has_any_cmd_sent:
            force_send = True
        
        # 非强制发送时的限制
        if not force_send:
            # 最小发送周期限制
            if (now - self._last_cmd_t).to_sec() < self.cmd_min_period_s:
                rospy.logdebug("Command skipped: min period not reached")
                return
            
            # 位置变化太小且速度相同
            if self._last_cmd_pos is not None and \
               abs(width_mm - self._last_cmd_pos) < self.pos_eps_mm and \
               self._last_cmd_spd == speed_mm_s:
                rospy.logdebug("Command skipped: position change too small")
                return
        
        # 创建并发送命令
        cmd = Cmd()
        cmd.pos = width_mm
        cmd.speed = speed_mm_s
        self.cmd_pub.publish(cmd)
        
        # 更新命令历史
        self._last_cmd_pos = width_mm
        self._last_cmd_spd = speed_mm_s
        self._last_cmd_t = now
        self._has_any_cmd_sent = True
        
        rospy.loginfo("Command sent: pos=%.2f mm, speed=%.1f mm/s", width_mm, speed_mm_s)

    # 未接触靠近：按节拍、近接自适应
    def _approach_step_close(self):
        """执行靠近步骤（使用最后有效状态）"""
        now = rospy.Time.now()
        
        # 检查发送周期
        if (now - self.last_approach_cmd_t).to_sec() < self.approach_cmd_period:
            return
        
        # 1. 确定当前有效宽度（优先级：当前状态 > 最后命令 > 起始宽度）
        if self.current_width is not None:
            current_width = self.current_width
        elif self._last_cmd_pos is not None:
            current_width = self._last_cmd_pos
        else:
            current_width = self.start_width
        
        # 2. 自适应步长和速度（近接触时减速）
        step = self.approach_step
        speed = self.approach_speed
        
        if self.meas_force_f is not None and \
           self.meas_force_f >= self.near_contact_ratio * self.contact_on_threshold:
            step = self.approach_step_near
            speed = self.approach_speed_near
        
        # 3. 计算并发送新位置（向闭合方向移动）
        new_width = clamp(current_width - step, self.min_width, self.max_width)
        self._send_goal(new_width, speed)
        
        # 更新最后命令时间
        self.last_approach_cmd_t = now
        rospy.loginfo("Approach step: %.2f mm -> %.2f mm (step: %.3f mm)", 
                      current_width, new_width, step)

    # 力环单步（位置增量）
    def _force_pid_step(self, dt):
        """执行力控PID步骤（使用最后有效状态）"""
        # 检查必要数据
        if self.current_width is None:
            rospy.logwarn_throttle(1.0, "Skipping force control: no width data")
            return
            
        if self.meas_force_f is None:
            rospy.logwarn_throttle(1.0, "Skipping force control: no force data")
            return
            
        if self.target_force is None:
            rospy.logwarn_throttle(1.0, "Skipping force control: no target force")
            return
        
        # 计算误差
        err = self.target_force - self.meas_force_f
        if abs(err) < self.deadband: 
            err = 0.0
        
        # I 项（带抗饱和）
        if self.ki > 0.0:
            self.int_acc += err * dt
            # 积分限幅
            i_max = self.i_limit / self.ki
            self.int_acc = clamp(self.int_acc, -i_max, i_max)
        else:
            self.int_acc = 0.0
        
        # D 项
        d_term = 0.0
        if self.prev_err is not None and dt > 1e-6:
            d_term = self.kd * (err - self.prev_err) / dt
        self.prev_err = err
        
        # PID 输出
        delta_mm = self.kp * err + self.ki * self.int_acc + d_term
        # 输出限幅
        delta_mm = clamp(delta_mm, -self.pid_step_limit_mm, self.pid_step_limit_mm)
        
        # 计算新宽度（向闭合方向移动）
        new_width = clamp(self.current_width - delta_mm, self.min_width, self.max_width)
        
        # 发送命令
        self._send_goal(new_width, self.pid_speed)
        rospy.loginfo("Force control: F_target=%.2fN, F_meas=%.2fN, delta=%.3fmm, width=%.2fmm",
                      self.target_force, self.meas_force_f, delta_mm, new_width)

    # 状态机主循环
    def spin(self):
        """状态机主循环"""
        rate = rospy.Rate(self.rate_hz)
        in_contact = False  # 当前接触状态
        self.phase_enter_t = rospy.Time.now()
        
        rospy.loginfo("Starting control loop at %.1f Hz", self.rate_hz)
        
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max((now - self.prev_t).to_sec(), 1e-6)
            self.prev_t = now
            
            # 状态机处理
            if self.phase == Phase.INIT:
                # 初始状态：发送起始位置命令
                self._send_goal(self.start_width, self.approach_speed, force_send=True)
                self.phase = Phase.AWAKE
                self.phase_enter_t = now
                rospy.loginfo("Transition: INIT -> AWAKE")
                
            elif self.phase == Phase.AWAKE:
                # 等待状态：等待第一帧状态数据
                awake_duration = (now - self.phase_enter_t).to_sec()
                
                if self._status_rx:
                    # 收到状态，进入靠近阶段
                    self.phase = Phase.APPROACH
                    self.phase_enter_t = now
                    rospy.loginfo("Transition: AWAKE -> APPROACH (status received)")
                elif awake_duration >= self.awake_timeout_s:
                    # 超时未收到状态，使用最后命令或默认值
                    rospy.logwarn("AWAKE timeout (%.1fs), proceeding without recent status", 
                                 awake_duration)
                    self.phase = Phase.APPROACH
                    self.phase_enter_t = now
                    rospy.loginfo("Transition: AWAKE -> APPROACH (timeout)")
                
            elif self.phase == Phase.APPROACH:
                # 靠近阶段：向目标移动直到接触
                # 检查是否达到接触阈值
                if self.meas_force_f is not None and self.meas_force_f >= self.contact_on_threshold:
                    # 进入力控阶段
                    self.int_acc = 0.0
                    self.prev_err = None
                    in_contact = True
                    self._publish_contact(True)
                    self.phase = Phase.FORCE
                    self.phase_enter_t = now
                    rospy.loginfo("Transition: APPROACH -> FORCE (contact detected)")
                else:
                    # 维持靠近状态
                    if in_contact:
                        in_contact = False
                        self._publish_contact(False)
                    
                    # 执行靠近步骤（使用最后有效状态）
                    self._approach_step_close()
                
            elif self.phase == Phase.FORCE:
                # 力控阶段：维持目标接触力
                # 检查是否失去接触
                if self.meas_force_f is not None and self.meas_force_f <= self.contact_off_threshold:
                    # 返回靠近阶段
                    if in_contact:
                        in_contact = False
                        self._publish_contact(False)
                    
                    self.int_acc = 0.0
                    self.prev_err = None
                    self.phase = Phase.APPROACH
                    self.phase_enter_t = now
                    rospy.loginfo("Transition: FORCE -> APPROACH (contact lost)")
                else:
                    # 维持力控状态
                    if not in_contact:
                        in_contact = True
                        self._publish_contact(True)
                    
                    # 执行力控步骤（使用最后有效状态）
                    self._force_pid_step(dt)
            
            # 控制循环结束
            rate.sleep()

if __name__ == "__main__":
    try:
        controller = WSG50ForcePIDSwitchSM()
        controller.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller shutdown by ROSInterrupt")
    except Exception as e:
        rospy.logerr("Unexpected error: %s", str(e))
