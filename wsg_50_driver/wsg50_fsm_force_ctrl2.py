#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WSG-50 三态有限状态机（INIT / APPROACH / FORCE）
- 输入 's' + 回车：INIT -> APPROACH
- APPROACH：低速按 110→0 方向闭合；若实测力 >= 阈值 -> FORCE
- FORCE：按目标力做 PID；若实测力 < 阈值 -> APPROACH
- 频率固定 30 Hz
- 订阅/发布话题沿用之前：
  * pub  : /wsg_50_driver/goal_position (wsg_50_common/Cmd)
  * sub  : /wsg_50_driver/status (wsg_50_common/Status: width[mm])  
  * sub  : /znsv6_data_sensor2      (实测力：支持 *WithHeader/Float32/Float64/MultiArray)
  * sub  : /znsv6_cmd/act2          (目标力：同上)
"""

import sys, select, threading, time
import rospy
from rospy import AnyMsg
from roslib.message import get_message_class
from wsg_50_common.msg import Cmd, Status
# [MOD] 绘图依赖（无显示环境用 Agg）
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except Exception:
    plt = None  # 如果 matplotlib 不可用，就在退出时提示但不报错
# [MOD] 路径/时间
import os, time
import numpy as np


def clamp(x, lo, hi): return max(lo, min(hi, x))

def extract_scalar_from_msg(msg, index=0):
    # 支持: Float32/Float64 取 data；MultiArray/WithHeader 取 data[index]
    if hasattr(msg, "data") and isinstance(msg.data, (float, int)):
        return float(msg.data)
    if hasattr(msg, "data") and hasattr(msg.data, "__len__") and len(msg.data) > 0:
        idx = index if 0 <= index < len(msg.data) else 0
        return float(msg.data[idx])
    return None

class WSG50FSM(object):
    def __init__(self):
        rospy.init_node("wsg50_fsm_force_ctrl", anonymous=False)

        # ---- 话题名（与之前一致，可用 _xxx:= 重映射）----
        self.goal_position_topic   = rospy.get_param("~goal_position_topic", "/wsg_50_driver/goal_position")
        self.status_topic          = rospy.get_param("~status_topic",        "/wsg_50_driver/status")
        self.measured_force_topic  = rospy.get_param("~measured_force_topic","/znsv6_data_sensor2")
        self.target_force_topic    = rospy.get_param("~target_force_topic",  "/znsv6_cmd/act2")

        # ---- 参数（仅必要的）----
        self.measured_force_index  = int(rospy.get_param("~measured_force_index", 0))
        self.target_force_index    = int(rospy.get_param("~target_force_index",   0))
        self.force_threshold_N     = float(rospy.get_param("~force_threshold_N",  0.1))  # 单阈值，双向判定

        self.approach_speed_mm_s   = float(rospy.get_param("~approach_speed_mm_s", 10.0))
        self.pid_speed_mm_s        = float(rospy.get_param("~pid_speed_mm_s",      10.0))
        self.start_width_mm        = float(rospy.get_param("~start_width_mm",     110.0))

        # PID（单位：mm/N, mm/(N·s), mm/(N·s)）
        self.kp = float(rospy.get_param("~kp_mm_per_N",   0.15))
        self.ki = float(rospy.get_param("~ki_mm_per_Ns",  0.00))
        self.kd = float(rospy.get_param("~kd_mm_per_Ns",  0.00))

        # 幅度/速度限（WSG-50 物理）
        self.min_width_mm   = float(rospy.get_param("~min_width_mm",   0.0))
        self.max_width_mm   = float(rospy.get_param("~max_width_mm", 110.0))
        self.min_speed_mm_s = float(rospy.get_param("~min_speed_mm_s",  5.0))
        self.max_speed_mm_s = float(rospy.get_param("~max_speed_mm_s", 420.0))

        # ---------- 发送去抖 & 死区 ----------
        self.pos_eps_mm = float(rospy.get_param("~pos_eps_mm", 0.03))
        self.rate_hz = 30.0
        self.cmd_min_period_s = float(rospy.get_param("~cmd_min_period_s", 1.0/self.rate_hz * 0.8))
        self._last_send_w = None
        self._last_send_v = None
        self._last_send_t = rospy.Time(0)

        # ---------- 积分启用门限 & 反风up ----------
        self.i_enable_band_N = float(rospy.get_param("~i_enable_band_N", 0.4))
        self.i_limit_mm = float(rospy.get_param("~i_limit_mm", 2.0))

        # ---------- 目标带死区 ----------
        self.target_deadband_N = float(rospy.get_param("~target_deadband_N", 0.1))

        # ---------- 新增：性能指标参数 ----------
        self.rise_frac = float(rospy.get_param("~rise_frac", 0.9))               # 上升百分比（默认 90%）
        self.settle_band_frac = float(rospy.get_param("~settle_band_frac", 0.1)) # 调整带宽（默认 ±10%）
        # ---------- 目标力缩放（控制使用缩放值，显示/日志用原始值） ----------
        self.target_scale = float(rospy.get_param("~target_scale", 1.15))

        # ---- 运行时变量 ----
        self.state   = "INIT"                       # INIT / APPROACH / FORCE
        self.prev_state = None

        self.width_mm = None
        self.pos_cmd  = None
        self.target_force = None
        self.meas_force   = None

        self._meas_cls = None
        self._tgt_cls  = None

        # PID 内部
        self.int_acc = 0.0
        self.prev_err = None

        # ---------- 新增：FORCE 期间的数据记录 ----------
        self._t_force_start = None
        self._force_log = []      # [(t_rel, |meas|, |tgt|), ...]
        self._rise_time_s = None
        self._settle_time_s = None

        # ---- ROS 通信 ----
        self.pub_cmd = rospy.Publisher(self.goal_position_topic, Cmd, queue_size=10)
        rospy.Subscriber(self.status_topic,         Status,  self._status_cb,  queue_size=20)
        rospy.Subscriber(self.measured_force_topic, AnyMsg,  self._meas_cb,    queue_size=50)
        rospy.Subscriber(self.target_force_topic,   AnyMsg,  self._target_cb,  queue_size=20)

        # ---- 键盘线程（等待 's'+回车）----
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        # ---------- 新增：退出时计算指标 ----------
        rospy.on_shutdown(self._on_shutdown)
        # [MOD] 保存曲线图的开关与路径（不传也能跑）
        self.save_plot = rospy.get_param("~save_plot", True)          # True=退出时保存曲线图
        self.plot_path = rospy.get_param("~plot_path", "")            # 为空则自动生成文件名到当前目录
        self.model_name = rospy.get_param("~model_name", rospy.get_param("/tac_policy_model_name", ""))

        rospy.loginfo("FSM ready. Press 's' + Enter to start APPROACH.")

    # ===== 回调 =====
    def _status_cb(self, msg: Status):
        self.width_mm = float(msg.width)

    def _meas_cb(self, any_msg: AnyMsg):
        try:
            if self._meas_cls is None:
                typ = any_msg._connection_header.get('type','')
                self._meas_cls = get_message_class(typ)
            m = self._meas_cls(); m.deserialize(any_msg._buff)
            v = extract_scalar_from_msg(m, self.measured_force_index)
            if v is not None: self.meas_force = float(v)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "measured_force parse failed: %s", e)

    def _target_cb(self, any_msg: AnyMsg):
        try:
            if self._tgt_cls is None:
                typ = any_msg._connection_header.get('type','')
                self._tgt_cls = get_message_class(typ)
            m = self._tgt_cls(); m.deserialize(any_msg._buff)
            v = extract_scalar_from_msg(m, self.target_force_index)
            if v is not None: self.target_force = float(v)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "target_force parse failed: %s", e)

    # ===== 键盘输入：'s' + Enter 进入 APPROACH =====
    def _keyboard_loop(self):
        while not rospy.is_shutdown():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.05); continue
                if line.strip().lower() == 's':
                    self.state = "APPROACH"
                    self.pos_cmd = None
                    rospy.loginfo("Key 's' pressed: INIT -> APPROACH")

    # ===== 发送命令（最小周期 + 位置死区）=====
    def _send_goal(self, width_mm, speed_mm_s):
        now = rospy.Time.now()
        w = clamp(width_mm, self.min_width_mm, self.max_width_mm)
        v = clamp(abs(speed_mm_s), self.min_speed_mm_s, self.max_speed_mm_s)

        if (now - self._last_send_t).to_sec() < self.cmd_min_period_s:
            return
        if self._last_send_w is not None and abs(w - self._last_send_w) < self.pos_eps_mm \
           and self._last_send_v == v:
            return

        cmd = Cmd(); cmd.pos = w; cmd.speed = v
        self.pub_cmd.publish(cmd)
        self._last_send_w, self._last_send_v, self._last_send_t = w, v, now

    # ===== 一个 tick（30Hz） =====
    def _tick(self, dt):
        # 进入 APPROACH / FORCE 时的处理
        if self.state != self.prev_state:
            if self.state == "APPROACH":
                base = self.width_mm if self.width_mm is not None else self.start_width_mm
                self.pos_cmd = clamp(base, self.min_width_mm, self.max_width_mm)
                rospy.loginfo("Enter APPROACH. init pos_cmd=%.2f mm", self.pos_cmd)
            elif self.state == "FORCE":
                self.int_acc = 0.0
                self.prev_err = None
                # ---- 新增：FORCE 日志复位并标记起点 ----
                self._t_force_start = rospy.Time.now()
                self._force_log = []
                self._rise_time_s = None
                self._settle_time_s = None
                rospy.loginfo("Enter FORCE.")
            self.prev_state = self.state

        # 状态逻辑
        if self.state == "INIT":
            return

        elif self.state == "APPROACH":
            if self.pos_cmd is None:
                base = self.width_mm if self.width_mm is not None else self.start_width_mm
                self.pos_cmd = clamp(base, self.min_width_mm, self.max_width_mm)

            step = self.approach_speed_mm_s / self.rate_hz
            self.pos_cmd = clamp(self.pos_cmd - step, self.min_width_mm, self.max_width_mm)
            self._send_goal(self.pos_cmd, self.approach_speed_mm_s)

            if (self.meas_force is not None) and (self.meas_force >= self.force_threshold_N):
                self.state = "FORCE"

        elif self.state == "FORCE":
            if (self.meas_force is None) or (self.meas_force < self.force_threshold_N):
                self.state = "APPROACH"
                return

            if self.target_force is None or self.width_mm is None:
                return

            # 控制用目标力：原始目标 * target_scale，再应用死区
            raw_target = self.target_force
            scaled_target = self.target_scale * raw_target
            tgt = 0.0 if abs(scaled_target) <= self.target_deadband_N else scaled_target
            # PID（误差: 缩放后目标力 - 实测力）
            err = tgt - self.meas_force

            # 积分启用门限 & 反风up限幅（以位移单位限制积分项）
            if abs(err) <= self.i_enable_band_N:
                self.int_acc += err * dt
            i_term = self.ki * self.int_acc
            if i_term > self.i_limit_mm:  i_term = self.i_limit_mm
            if i_term < -self.i_limit_mm: i_term = -self.i_limit_mm

            d_term = 0.0 if self.prev_err is None else self.kd * (err - self.prev_err) / max(dt, 1e-6)
            self.prev_err = err

            delta_mm = self.kp*err + i_term + d_term
            new_width = clamp(self.width_mm - delta_mm, self.min_width_mm, self.max_width_mm)
            print("FORCE CTRL: target_raw=%.3f N, target_ctrl=%.3f N, meas=%.3f N, err=%.3f N ,new_width=%.2f mm"
                  % (raw_target, tgt, self.meas_force, err, new_width))
            self._send_goal(new_width, self.pid_speed_mm_s)

            # ---- 新增：FORCE 期间记录（相对 FORCE 起点时间）----
            if self._t_force_start is not None and self.meas_force is not None:
                t_rel = (rospy.Time.now() - self._t_force_start).to_sec()
                # 日志中保留原始目标力（绝对值），显示/指标用原始值
                self._force_log.append( (t_rel, abs(self.meas_force), abs(raw_target)) )

    # ===== 退出时计算上升时间 / 调整时间 =====
    def _on_shutdown(self):
        try:
            if not self._force_log:
                rospy.loginfo(
                    "No FORCE-phase data; PID[Kp=%.3f, Ki=%.3f, Kd=%.3f] – no times.",
                    getattr(self, "kp", float("nan")),
                    getattr(self, "ki", float("nan")),
                    getattr(self, "kd", float("nan")),
                )
                return

            # 解包记录：ts[0..N-1], meas_abs[k], tgt_abs[k]
            ts, meas_abs, tgt_abs = zip(*self._force_log)  # note: keep your original attr name
            n = len(meas_abs)

            # ---- 上升时间：相对稳态值 Yss 的 ~self.rise_frac ----
            Yss = meas_abs[-1]
            if Yss <= 1e-9:
                rospy.loginfo(
                    "Steady-state force ~0; cannot compute times. PID[Kp=%.3f, Ki=%.3f, Kd=%.3f]",
                    getattr(self, "kp", float("n an")), getattr(self, "ki", float("nan")), getattr(self, "kd", float("nan"))
                )
                return

            thr_rise = self.rise_frac * Yss
            k_rise = None
            for i, y in enumerate(meas_abs):
                if y >= thr_rise:
                    k_rise = i
                    break
            if k_rise is None:
                # 连达到阈值都没有
                kp = getattr(self, "kp", float("nan"))
                ki = getattr(self, "ki", float("nan"))
                kd = getattr(self, "kd", float("nan"))
                tail_rng = max(meas_abs) - min(meas_abs)
                rospy.loginfo(
                    "PID[Kp=%.3f, Ki=%.3f, Kd=%.3f]; t_rise=n/a; t_peak=n/a; t_settle=n/a "
                    "(never reached %.0f%% of Yss=%.3f N; tail range %.3f N)",
                    kp, ki, kd, self.rise_frac * 100.0, Yss, tail_rng
                )
                return


            # ---- 峰值时间 t_peak：从 k_rise 之后寻找最大值出现的时刻 ----
            peak_val = max(meas_abs[k_rise:])
            peak_idx = k_rise + next(j for j in range(0, n - k_rise) if meas_abs[k_rise + j] == peak_val)
            t_peak = ts[peak_idx]

            # ---- 稳态时间（按“后缀波动 ≤ settle_band_frac*|target|”）----
            Ttgt = tgt_abs[-1] if tgt_abs[-1] > 1e-9 else max(tgt_abs)
            if Ttgt <= 1e-9:
                Ttgt = Yss  # 防止目标为 0 时分母为 0
            band_abs = self.settle_band_frac * Ttgt

            # 预计算后缀区间的最大/最小值，O(n) 求任意后缀的 (max-min)
            tail_max = [0.0] * n
            tail_min = [0.0] * n
            tail_max[-1] = meas_abs[-1]
            tail_min[-1] = meas_abs[-1]
            for i in range(n - 2, -1, -1):
                v = meas_abs[i]
                tail_max[i] = v if v > tail_max[i+1] else tail_max[i+1]
                tail_min[i] = v if v < tail_min[i+1] else tail_min[i+1]

            t_settle_str = "n/a"
            if k_rise is not None:
                settle_idx = None
                for i in range(k_rise, n):
                    rng = tail_max[i] - tail_min[i]
                    if rng <= band_abs:
                        settle_idx = i
                        break
                if settle_idx is not None:
                    t_settle_str = f"{ts[settle_idx]:.3f}s(≤{self.settle_band_frac*100:.0f}%%·|T|)"
                else:
                    # 给出最终尾段的实际波动，便于评估
                    final_rng = tail_max[k_rise] - tail_min[k_rise]
                    t_settle_str = f"n/a(Δtail={final_rng:.3f}N>{band_abs:.3f}N)"

            # ---- 单行输出：先 PID，再三个时间指标 ----
            kp = getattr(self, "kp", float("nan"))
            ki = getattr(self, "ki", float("nan"))
            kd = getattr(self, "kd", float("nan"))
            rospy.loginfo(
                "PID[Kp=%.3f, Ki=%.3f, Kd=%.3f]; "
                "t_rise=%.3fs(@%.3fN≈%.0f%%·Yss=%.3fN); "
                "t_peak=%.3fs(@Fmax=%.3fN); "
                "t_settle=%s",
                kp, ki, kd,
                ts[k_rise], thr_rise, self.rise_frac*100.0, Yss,
                t_peak, peak_val,
                t_settle_str
            )
            # ---- 新增：保存曲线图 ----
            if plt is not None and self.save_plot:
                try:
                    plt.figure(figsize=(8,5))
                    plt.plot(ts, meas_abs, label="Measured Force |F| (N)", color='blue')
                    plt.plot(ts, tgt_abs, label="Target Force |T| (N)", color='orange', linestyle='--')

                    plt.axhline(y=thr_rise, color='green', linestyle=':', label=f'Rise Threshold ({self.rise_frac*100:.0f}% Yss)')
                    # plt.axhline(y=Yss*(1+self.settle_band_frac), color='red', linestyle='-.', label='Settle Band Upper')
                    # plt.axhline(y=Yss*(1-self.settle_band_frac), color='red', linestyle='-.', label='Settle Band Lower')
                    plt.title("WSG-50 FORCE Control Performance")
                    model_name = self.model_name or rospy.get_param("/tac_policy_model_name", "")
                    if model_name:
                        plt.gcf().text(0.01, 0.01, "Model: {}".format(model_name), fontsize=8, ha="left", va="bottom")
                    plt.xlabel("Time (s)")
                    plt.ylabel("Force (N)")
                    plt.legend()
                    plt.grid(True)
                    if not self.plot_path:
                        timestamp = time.strftime("%Y%m%d_%H%M%S")
                        self.plot_path = os.path.join(os.getcwd(), f"wsg50_force_ctrl_{timestamp}.png")
                    plt.savefig(self.plot_path)
                    rospy.loginfo("Force control plot saved to: %s", self.plot_path)
                except Exception as e:
                    rospy.logwarn("Failed to save plot: %s", e)
        except Exception as e:
            rospy.logwarn("Metrics computation failed: %s", e)





    # ===== 主循环 =====
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        prev_t = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - prev_t).to_sec()
            if dt <= 0: dt = 1.0/self.rate_hz
            prev_t = now
            self._tick(dt)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = WSG50FSM()
        node.spin()
    except rospy.ROSInterruptException:
        pass
