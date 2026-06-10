#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WSG-50 五态有限状态机（INIT / APPROACH / FORCE / OPEN_TO_START / WAIT_REAPPROACH）
- 输入 's' + 回车：INIT/WAIT_REAPPROACH -> APPROACH
- APPROACH：低速按 110→0 方向闭合；若实测力 >= 阈值 -> FORCE
- FORCE：按目标力做 PID；目标力快速下降或失接触 -> OPEN_TO_START
- OPEN_TO_START：张开到 start_width_mm；到位或超时 -> WAIT_REAPPROACH
- WAIT_REAPPROACH：保持张开，等待键盘 's'
- 频率固定 30 Hz
- 订阅/发布话题沿用之前：
  * pub  : /wsg_50_driver/goal_position (wsg_50_common/Cmd)
  * pub  : ~/measured_force_filtered, ~/target_force_filtered (std_msgs/Float32)
  * sub  : /wsg_50_driver/status (wsg_50_common/Status: width[mm])  
  * sub  : /znsv6_data_sensor2      (实测力：支持 *WithHeader/Float32/Float64/MultiArray)
  * sub  : /znsv6_cmd/act2          (目标力：同上)
"""

import sys, select, threading, time
import csv
import math
from collections import deque
import rospy
from rospy import AnyMsg
from roslib.message import get_message_class
from wsg_50_common.msg import Cmd, Status
from std_msgs.msg import String, Float32
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


class HoldTimer(object):
    def __init__(self):
        self.start_s = None

    def reset(self):
        self.start_s = None

    def update(self, condition, now_s, duration_s):
        if not condition:
            self.start_s = None
            return False
        if self.start_s is None:
            self.start_s = now_s
        return (now_s - self.start_s) >= max(0.0, duration_s)


class TargetTrendDetector(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.samples = deque()
        self.filtered = None
        self.last_t_s = None
        self.confirm_start_s = None
        self.cooldown_until_s = 0.0
        self.last_metrics = {}
        self.last_confirm_time_s = None

    def _median(self, values):
        if not values:
            return float("nan")
        vals = sorted(values)
        n = len(vals)
        mid = n // 2
        if n % 2:
            return vals[mid]
        return 0.5 * (vals[mid - 1] + vals[mid])

    def _slope(self, xs, ys):
        n = len(xs)
        if n < 2:
            return 0.0
        mx = sum(xs) / n
        my = sum(ys) / n
        den = sum((x - mx) ** 2 for x in xs)
        if den <= 1e-12:
            return 0.0
        return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den

    def _compute_metrics(self):
        n = len(self.samples)
        if n < self.cfg["min_samples"]:
            return None

        xs = [p[0] for p in self.samples]
        ys = [p[1] for p in self.samples]
        duration_s = xs[-1] - xs[0]
        if duration_s < self.cfg["min_window_s"]:
            return None

        q = max(1, n // 4)
        start_level = self._median(ys[:q])
        end_level = self._median(ys[-q:])
        drop_N = start_level - end_level
        drop_frac = drop_N / max(start_level, 1e-9)
        slope_N_per_s = self._slope(xs, ys)

        jitter = self.cfg["jitter_deadband_N"]
        diffs = [ys[i] - ys[i - 1] for i in range(1, n)]
        valid_diffs = [d for d in diffs if abs(d) > jitter]
        neg_mag = sum(-d for d in valid_diffs if d < 0.0)
        pos_mag = sum(d for d in valid_diffs if d > 0.0)
        mag_total = neg_mag + pos_mag
        path = sum(abs(d) for d in valid_diffs)
        neg_mag_ratio = neg_mag / mag_total if mag_total > 1e-12 else 0.0
        efficiency = drop_N / path if path > 1e-12 else 0.0

        return {
            "n_samples": n,
            "window_duration_s": duration_s,
            "start_level_N": start_level,
            "end_level_N": end_level,
            "drop_N": drop_N,
            "drop_frac": drop_frac,
            "slope_N_per_s": slope_N_per_s,
            "neg_mag_ratio": neg_mag_ratio,
            "efficiency": efficiency,
        }

    def update_and_check(self, t_s, target_abs_N):
        target_abs_N = max(0.0, float(target_abs_N))
        if self.filtered is None or self.last_t_s is None:
            self.filtered = target_abs_N
        else:
            dt = max(0.0, t_s - self.last_t_s)
            tau = self.cfg["filter_tau_s"]
            alpha = 1.0 if tau <= 1e-9 else 1.0 - math.exp(-dt / tau)
            alpha = clamp(alpha, 0.0, 1.0)
            self.filtered = alpha * target_abs_N + (1.0 - alpha) * self.filtered
        self.last_t_s = t_s

        self.samples.append((t_s, self.filtered))
        while self.samples and (t_s - self.samples[0][0]) > self.cfg["window_s"]:
            self.samples.popleft()

        metrics = self._compute_metrics()
        if metrics is None:
            self.confirm_start_s = None
            self.last_metrics = {
                "n_samples": len(self.samples),
                "window_duration_s": (self.samples[-1][0] - self.samples[0][0]) if len(self.samples) >= 2 else 0.0,
                "falling": False,
                "confirmed": False,
            }
            return False, self.last_metrics

        falling = (
            metrics["start_level_N"] >= self.cfg["min_start_N"] and
            metrics["drop_N"] >= self.cfg["min_drop_N"] and
            metrics["drop_frac"] >= self.cfg["min_drop_frac"] and
            metrics["slope_N_per_s"] <= -self.cfg["min_slope_N_per_s"] and
            metrics["neg_mag_ratio"] >= self.cfg["min_neg_mag_ratio"] and
            metrics["efficiency"] >= self.cfg["min_efficiency"]
        )

        if falling:
            if self.confirm_start_s is None:
                self.confirm_start_s = t_s
            confirmed = (
                t_s >= self.cooldown_until_s and
                (t_s - self.confirm_start_s) >= self.cfg["confirm_s"]
            )
        else:
            self.confirm_start_s = None
            confirmed = False

        if confirmed:
            self.cooldown_until_s = t_s + self.cfg["cooldown_s"]
            self.last_confirm_time_s = t_s

        metrics["falling"] = falling
        metrics["confirmed"] = confirmed
        self.last_metrics = metrics
        return confirmed, metrics

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
        self.force_threshold_N     = float(rospy.get_param("~force_threshold_N",  0.15))  # 单阈值，双向判定

        # 力信号预处理：缩放 + 负数归零 + 低通滤波（EMA）
        self.measured_scale         = float(rospy.get_param("~measured_scale", 1.0))
        self.force_lpf_alpha        = float(rospy.get_param("~force_lpf_alpha", 0.3))
        self.target_force_lpf_alpha = float(rospy.get_param("~target_force_lpf_alpha", self.force_lpf_alpha))
        self.target_timeout_s       = float(rospy.get_param("~target_timeout_s", 1.0))

        # v3: stale / sensor fault 策略
        self.target_stale_timeout_s = float(rospy.get_param("~target_stale_timeout_s", 0.30))
        self.meas_stale_timeout_s   = float(rospy.get_param("~meas_stale_timeout_s",   0.30))
        self.status_stale_timeout_s = float(rospy.get_param("~status_stale_timeout_s", 0.50))
        self.sensor_fault_policy    = str(rospy.get_param("~sensor_fault_policy", "hold_no_new_cmd"))
        self.reset_pid_on_stale     = bool(rospy.get_param("~reset_pid_on_stale", True))
        self.sensor_fault_open_enable = bool(rospy.get_param("~sensor_fault_open_enable", False))

        self.approach_speed_mm_s   = float(rospy.get_param("~approach_speed_mm_s", 10.0))
        self.pid_speed_mm_s        = float(rospy.get_param("~pid_speed_mm_s",      10.0))
        self.start_width_mm        = float(rospy.get_param("~start_width_mm",     110.0))

        # v3: 张开 / 保持张开
        self.manual_reapproach_only = bool(rospy.get_param("~manual_reapproach_only", True))
        self.open_speed_mm_s = float(rospy.get_param("~open_speed_mm_s", 50.0))
        self.hold_open_speed_mm_s = float(rospy.get_param("~hold_open_speed_mm_s", 30.0))
        self.open_width_tol_mm = float(rospy.get_param("~open_width_tol_mm", 2.0))
        self.open_min_hold_s = float(rospy.get_param("~open_min_hold_s", 0.25))
        self.open_timeout_s = float(rospy.get_param("~open_timeout_s", 3.0))
        self.open_command_force_resend_period_s = float(rospy.get_param("~open_command_force_resend_period_s", 0.30))
        self.hold_open_command_period_s = float(rospy.get_param("~hold_open_command_period_s", 0.30))
        self.warn_on_low_target_reapproach = bool(rospy.get_param("~warn_on_low_target_reapproach", True))
        self.low_target_reapproach_warn_N = float(rospy.get_param("~low_target_reapproach_warn_N", 0.4))

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

        # v3: 目标力下降趋势触发张开
        self.release_gate_mode = str(rospy.get_param("~release_gate_mode", "trend_only")).strip().lower()
        allowed_release_modes = ("trend_only", "trend_and_target_low", "trend_and_low_force", "trend_and_release_shape")
        if self.release_gate_mode not in allowed_release_modes:
            rospy.logwarn("Unknown release_gate_mode=%r, using 'trend_only'", self.release_gate_mode)
            self.release_gate_mode = "trend_only"
        self.release_intent_timeout_s = float(rospy.get_param("~release_intent_timeout_s", 1.5))
        self.trend_release_require_meas_valid = bool(rospy.get_param("~trend_release_require_meas_valid", True))
        self.trend_release_require_status_valid = bool(rospy.get_param("~trend_release_require_status_valid", False))

        self.trend_open_enable = bool(rospy.get_param("~trend_open_enable", True))
        self.trend_window_s = float(rospy.get_param("~trend_window_s", 0.6))
        self.trend_min_window_s = float(rospy.get_param("~trend_min_window_s", 0.35))
        self.trend_min_samples_auto = bool(rospy.get_param("~trend_min_samples_auto", False))
        self.target_cmd_hz_est = float(rospy.get_param("~target_cmd_hz_est", 30.0))
        default_min_samples = max(5, int(math.ceil(self.trend_min_window_s * self.target_cmd_hz_est))) \
            if self.trend_min_samples_auto else 10
        self.trend_min_samples = int(rospy.get_param("~trend_min_samples", default_min_samples))
        self.trend_use_time_constant_filter = bool(rospy.get_param("~trend_use_time_constant_filter", True))
        self.trend_filter_tau_s = float(rospy.get_param("~trend_filter_tau_s", 0.08))
        self.trend_jitter_deadband_N = float(rospy.get_param("~trend_jitter_deadband_N", 0.05))
        self.trend_min_start_N = float(rospy.get_param("~trend_min_start_N", 0.8))
        self.trend_min_drop_N = float(rospy.get_param("~trend_min_drop_N", 0.55))
        self.trend_min_drop_frac = float(rospy.get_param("~trend_min_drop_frac", 0.12))
        self.trend_min_slope_N_per_s = float(rospy.get_param("~trend_min_slope_N_per_s", 1.0))
        self.trend_min_neg_mag_ratio = float(rospy.get_param("~trend_min_neg_mag_ratio", 0.70))
        self.trend_min_efficiency = float(rospy.get_param("~trend_min_efficiency", 0.45))
        self.trend_confirm_s = float(rospy.get_param("~trend_confirm_s", 0.08))
        self.trend_cooldown_s = float(rospy.get_param("~trend_cooldown_s", 0.8))

        # v3: trend_only 的软保护，默认只 warning，不阻止当前高力区下降触发
        self.trend_shape_guard_mode = str(rospy.get_param("~trend_shape_guard_mode", "warn")).strip().lower()
        if self.trend_shape_guard_mode not in ("off", "warn", "enforce"):
            rospy.logwarn("Unknown trend_shape_guard_mode=%r, using 'warn'", self.trend_shape_guard_mode)
            self.trend_shape_guard_mode = "warn"
        self.trend_use_end_cap = bool(rospy.get_param("~trend_use_end_cap", False))
        self.trend_max_end_N = float(rospy.get_param("~trend_max_end_N", 2.0))
        self.trend_use_min_drop_frac_high_force = bool(rospy.get_param("~trend_use_min_drop_frac_high_force", False))
        self.trend_high_force_start_N = float(rospy.get_param("~trend_high_force_start_N", 3.0))
        self.trend_min_drop_frac_high_force = float(rospy.get_param("~trend_min_drop_frac_high_force", 0.18))

        # v3: 低力只做 debug 观察，默认不参与 trend_only 触发
        self.release_target_threshold_N = float(rospy.get_param("~release_target_threshold_N", 0.4))
        self.release_measured_threshold_N = float(rospy.get_param("~release_measured_threshold_N", 0.8))
        self.release_target_low_confirm_s = float(rospy.get_param("~release_target_low_confirm_s", 0.08))
        self.release_measured_low_confirm_s = float(rospy.get_param("~release_measured_low_confirm_s", 0.12))

        # v3: FORCE 中失接触后进入 OPEN_TO_START
        self.force_contact_lost_to_open_enable = bool(rospy.get_param("~force_contact_lost_to_open_enable", True))
        self.force_contact_lost_threshold_N = float(
            rospy.get_param("~force_contact_lost_threshold_N", 0.5 * self.force_threshold_N))
        self.force_contact_lost_grace_s = float(rospy.get_param("~force_contact_lost_grace_s", 0.25))
        self.force_contact_lost_confirm_s = float(rospy.get_param("~force_contact_lost_confirm_s", 0.30))

        # ---------- 新增：性能指标参数 ----------
        self.rise_frac = float(rospy.get_param("~rise_frac", 0.9))               # 上升百分比（默认 90%）
        self.settle_band_frac = float(rospy.get_param("~settle_band_frac", 0.1)) # 调整带宽（默认 ±10%）
        # ---------- 目标力缩放（控制使用缩放值，显示/日志用原始值） ----------
        self.target_scale = float(rospy.get_param("~target_scale", 1.15))

        # ---- 运行时变量 ----
        self._lock = threading.RLock()
        self.state   = "INIT"                       # INIT / APPROACH / FORCE / OPEN_TO_START / WAIT_REAPPROACH
        self.prev_state = None

        self.width_mm = None
        self.pos_cmd  = None
        self.status_stamp_s = None

        # raw / filtered forces (N)
        # - measured: scaled, then negative->0, then filter
        # - target  : abs, then filter; scaling仍在控制里统一乘 target_scale
        self.target_raw_signed_N = None
        self.target_abs_raw_N = None
        self.target_force_raw = None   # 兼容旧日志字段：abs(target) 未滤波
        self.target_force_f   = None   # abs(target) 低通后（未缩放）
        self.meas_raw_signed_N = None
        self.meas_scaled_nonneg_N = None
        self.meas_force_raw   = None   # 兼容旧日志字段：scaled + non-negative measured
        self.meas_force_f     = None   # measured_scale 后、负数归零 + 低通后

        self.target_force = None
        self.meas_force   = None

        self._meas_cls = None
        self._tgt_cls  = None
        self.last_target_rx_t = None
        self.target_stamp_s = None
        self.meas_stamp_s = None

        # 键盘请求由主 tick 消费，键盘线程不直接改状态
        self.reapproach_requested = False
        self.ignored_s_count = 0
        self.last_s_time_s = None
        self.last_ignored_s_state = None

        # PID 内部
        self.int_acc = 0.0
        self.prev_err = None
        self.prev_pid_time_s = None

        trend_cfg = {
            "window_s": self.trend_window_s,
            "min_window_s": self.trend_min_window_s,
            "min_samples": self.trend_min_samples,
            "filter_tau_s": self.trend_filter_tau_s if self.trend_use_time_constant_filter else 0.0,
            "jitter_deadband_N": self.trend_jitter_deadband_N,
            "min_start_N": self.trend_min_start_N,
            "min_drop_N": self.trend_min_drop_N,
            "min_drop_frac": self.trend_min_drop_frac,
            "min_slope_N_per_s": self.trend_min_slope_N_per_s,
            "min_neg_mag_ratio": self.trend_min_neg_mag_ratio,
            "min_efficiency": self.trend_min_efficiency,
            "confirm_s": self.trend_confirm_s,
            "cooldown_s": self.trend_cooldown_s,
        }
        self.target_trend_detector = TargetTrendDetector(trend_cfg)
        self.last_trend_target_stamp_s = None
        self.last_trend_metrics = {}
        self.release_intent_latched = False
        self.release_intent_until_s = 0.0
        self.release_intent_reason = ""
        self.release_trigger_time_s = None
        self.release_trigger_metrics = {}
        self.release_open_triggered = False
        self.trend_shape_warning = False

        self.target_low_timer = HoldTimer()
        self.measured_low_timer = HoldTimer()
        self.contact_lost_timer = HoldTimer()
        self.target_low_now = False
        self.target_low_confirmed = False
        self.measured_low_now = False
        self.measured_low_confirmed = False
        self.low_force_ok = False
        self.contact_lost_now = False
        self.contact_lost_confirmed = False
        self.contact_lost_in_grace = False
        self.contact_lost_open_triggered = False

        self.force_enter_time_s = None
        self.open_reason = ""
        self.open_enter_time_s = None
        self.wait_reapproach_enter_time_s = None
        self.open_failed = False
        self.opened_enough = False
        self.last_open_force_send_s = 0.0
        self.last_hold_open_send_s = 0.0
        self.last_cmd_width_mm = None
        self.last_cmd_speed_mm_s = None

        # ---------- 新增：FORCE 期间的数据记录 ----------
        self._t_force_start = None
        self._force_log = []      # [{ros_time_s, t_rel_s, measured_filtered_N, target_ctrl_N, ...}, ...]
        self._rise_time_s = None
        self._settle_time_s = None
        self._csv_saved = False

        # ---- ROS 通信 ----
        self.pub_cmd = rospy.Publisher(self.goal_position_topic, Cmd, queue_size=10)
        self.debug_topic = rospy.get_param("~debug_topic", "debug")
        self.debug_period_s = float(rospy.get_param("~debug_period_s", 0.2))
        self.pub_debug = rospy.Publisher(self.debug_topic, String, queue_size=10)
        self.filtered_measured_force_topic = rospy.get_param(
            "~filtered_measured_force_topic", "~measured_force_filtered")
        self.filtered_target_force_topic = rospy.get_param(
            "~filtered_target_force_topic", "~target_force_filtered")
        self.pub_meas_force_filtered = rospy.Publisher(
            self.filtered_measured_force_topic, Float32, queue_size=10)
        self.pub_target_force_filtered = rospy.Publisher(
            self.filtered_target_force_topic, Float32, queue_size=10)
        self._last_debug_t = rospy.Time(0)
        rospy.Subscriber(self.status_topic,         Status,  self._status_cb,  queue_size=20)
        rospy.Subscriber(self.measured_force_topic, AnyMsg,  self._meas_cb,    queue_size=50)
        rospy.Subscriber(self.target_force_topic,   AnyMsg,  self._target_cb,  queue_size=20)

        # ---- 键盘线程（等待 's'+回车）----
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        # ---------- 新增：退出时计算指标 ----------
        rospy.on_shutdown(self._on_shutdown)
        # [MOD] 保存曲线图的开关与路径（不传也能跑）
        self.save_plot = rospy.get_param("~save_plot", False)         # True=退出时保存曲线图
        self.plot_path = rospy.get_param("~plot_path", "")            # 为空则自动生成文件名到当前目录
        self.save_csv = rospy.get_param("~save_csv", True)            # True=退出时保存 FORCE 阶段 CSV
        self.csv_path = rospy.get_param("~csv_path", "")              # 为空则自动生成带时间戳文件名到当前目录
        self.model_name = rospy.get_param("~model_name", rospy.get_param("/tac_policy_model_name", ""))

        rospy.loginfo("FSM ready. Press 's' + Enter to start APPROACH.")

    # ===== 回调 =====
    def _status_cb(self, msg: Status):
        with self._lock:
            self.width_mm = float(msg.width)
            self.status_stamp_s = rospy.Time.now().to_sec()

    def _meas_cb(self, any_msg: AnyMsg):
        try:
            if self._meas_cls is None:
                typ = any_msg._connection_header.get('type','')
                self._meas_cls = get_message_class(typ)
            m = self._meas_cls(); m.deserialize(any_msg._buff)
            v = extract_scalar_from_msg(m, self.measured_force_index)
            if v is None:
                return

            raw_signed = float(v)
            f = raw_signed * self.measured_scale
            f = max(0.0, f)  # 负数归零（替代 abs）
            with self._lock:
                self.meas_raw_signed_N = raw_signed
                self.meas_scaled_nonneg_N = f
                self.meas_force_raw = f

                if self.meas_force_f is None:
                    self.meas_force_f = f
                else:
                    a = clamp(self.force_lpf_alpha, 0.0, 1.0)
                    self.meas_force_f = a * f + (1.0 - a) * self.meas_force_f

                self.meas_force = self.meas_force_f
                self.meas_stamp_s = rospy.Time.now().to_sec()
                pub_val = float(self.meas_force)
            self.pub_meas_force_filtered.publish(Float32(data=pub_val))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "measured_force parse failed: %s", e)

    def _target_cb(self, any_msg: AnyMsg):
        try:
            if self._tgt_cls is None:
                typ = any_msg._connection_header.get('type','')
                self._tgt_cls = get_message_class(typ)
            m = self._tgt_cls(); m.deserialize(any_msg._buff)
            v = extract_scalar_from_msg(m, self.target_force_index)
            if v is None:
                return
            now = rospy.Time.now()
            raw_signed = float(v)
            t = abs(raw_signed)  # 目标力取正（按原话题值）
            with self._lock:
                self.last_target_rx_t = now
                self.target_stamp_s = now.to_sec()
                self.target_raw_signed_N = raw_signed
                self.target_abs_raw_N = t
                self.target_force_raw = t

                if self.target_force_f is None:
                    self.target_force_f = t
                else:
                    a = clamp(self.target_force_lpf_alpha, 0.0, 1.0)
                    self.target_force_f = a * t + (1.0 - a) * self.target_force_f

                self.target_force = self.target_force_f
                pub_val = float(self.target_force)
            self.pub_target_force_filtered.publish(Float32(data=pub_val))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "target_force parse failed: %s", e)

    def _check_target_timeout(self):
        if self.target_timeout_s <= 0.0:
            return
        now = rospy.Time.now()
        with self._lock:
            last_target_rx_t = self.last_target_rx_t
        if last_target_rx_t is None:
            rospy.logwarn_throttle(
                1.0,
                "目标力超时未收到: topic=%s, timeout=%.2fs",
                self.target_force_topic,
                self.target_timeout_s,
            )
            return
        age = (now - last_target_rx_t).to_sec()
        if age > self.target_timeout_s:
            rospy.logwarn_throttle(
                1.0,
                "目标力超时未收到: topic=%s, age=%.2fs > %.2fs",
                self.target_force_topic,
                age,
                self.target_timeout_s,
            )

    # ===== 键盘输入：'s' + Enter 进入 APPROACH =====
    def _keyboard_loop(self):
        while not rospy.is_shutdown():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.05); continue
                if line.strip().lower() == 's':
                    with self._lock:
                        self.reapproach_requested = True
                        self.last_s_time_s = rospy.Time.now().to_sec()
                    rospy.loginfo("Key 's' pressed: request APPROACH")

    # ===== 发送命令（最小周期 + 位置死区）=====
    def _send_goal(self, width_mm, speed_mm_s, force=False):
        now = rospy.Time.now()
        w = clamp(width_mm, self.min_width_mm, self.max_width_mm)
        v = clamp(abs(speed_mm_s), self.min_speed_mm_s, self.max_speed_mm_s)

        if not force:
            if (now - self._last_send_t).to_sec() < self.cmd_min_period_s:
                return False
            if self._last_send_w is not None and abs(w - self._last_send_w) < self.pos_eps_mm \
               and self._last_send_v == v:
                return False

        cmd = Cmd(); cmd.pos = w; cmd.speed = v
        self.pub_cmd.publish(cmd)
        self._last_send_w, self._last_send_v, self._last_send_t = w, v, now
        self.last_cmd_width_mm = w
        self.last_cmd_speed_mm_s = v
        return True

    def _age_s(self, now_s, stamp_s):
        return float("inf") if stamp_s is None else max(0.0, now_s - stamp_s)

    def _make_snapshot(self, now_s):
        with self._lock:
            s = {
                "target_raw_signed_N": self.target_raw_signed_N,
                "target_abs_raw_N": self.target_abs_raw_N,
                "target_force_f_N": self.target_force_f,
                "target_stamp_s": self.target_stamp_s,
                "meas_raw_signed_N": self.meas_raw_signed_N,
                "meas_scaled_nonneg_N": self.meas_scaled_nonneg_N,
                "meas_force_f_N": self.meas_force_f,
                "meas_stamp_s": self.meas_stamp_s,
                "width_mm": self.width_mm,
                "status_stamp_s": self.status_stamp_s,
            }

        s["target_age_s"] = self._age_s(now_s, s["target_stamp_s"])
        s["meas_age_s"] = self._age_s(now_s, s["meas_stamp_s"])
        s["width_age_s"] = self._age_s(now_s, s["status_stamp_s"])
        s["target_valid"] = (
            s["target_abs_raw_N"] is not None and
            s["target_age_s"] <= self.target_stale_timeout_s
        )
        s["meas_valid"] = (
            s["meas_force_f_N"] is not None and
            s["meas_age_s"] <= self.meas_stale_timeout_s
        )
        s["width_valid"] = (
            s["width_mm"] is not None and
            s["width_age_s"] <= self.status_stale_timeout_s
        )
        return s

    def _take_reapproach_request(self):
        with self._lock:
            requested = self.reapproach_requested
            self.reapproach_requested = False
        return requested

    def _record_ignored_s(self):
        self.ignored_s_count += 1
        self.last_ignored_s_state = self.state

    def _reset_send_cache(self):
        self._last_send_t = rospy.Time(0)
        self._last_send_w = None
        self._last_send_v = None

    def _reset_pid_dynamic_state(self):
        self.int_acc = 0.0
        self.prev_err = None
        self.prev_pid_time_s = None

    def _set_state(self, new_state):
        self.prev_state = self.state
        self.state = new_state

    def _enter_approach(self, now_s, snapshot):
        self._set_state("APPROACH")
        if snapshot.get("width_valid"):
            base = snapshot["width_mm"]
        else:
            base = self.start_width_mm
        self.pos_cmd = clamp(base, self.min_width_mm, self.max_width_mm)
        self._reset_pid_dynamic_state()
        self.target_trend_detector.reset()
        self.contact_lost_timer.reset()
        self.target_low_timer.reset()
        self.measured_low_timer.reset()
        self.release_intent_latched = False
        self.release_intent_until_s = 0.0
        self.release_open_triggered = False
        self._reset_send_cache()
        rospy.loginfo("Enter APPROACH. init pos_cmd=%.2f mm", self.pos_cmd)

    def _enter_force(self, now_s, snapshot):
        self._set_state("FORCE")
        self.force_enter_time_s = now_s
        self._reset_pid_dynamic_state()
        self.target_trend_detector.reset()
        self.contact_lost_timer.reset()
        self.target_low_timer.reset()
        self.measured_low_timer.reset()
        self.last_trend_target_stamp_s = None
        self.release_intent_latched = False
        self.release_intent_until_s = 0.0
        self.release_open_triggered = False
        self.contact_lost_open_triggered = False
        self._t_force_start = rospy.Time.now()
        self._force_log = []
        self._rise_time_s = None
        self._settle_time_s = None
        rospy.loginfo("Enter FORCE.")

    def _enter_open_to_start(self, now_s, reason, snapshot, trend_metrics=None):
        self._set_state("OPEN_TO_START")
        self.open_reason = reason
        self.open_enter_time_s = now_s
        self.open_failed = False
        self.opened_enough = False
        self.release_trigger_time_s = now_s
        self.release_trigger_metrics = dict(trend_metrics or {})
        self._reset_pid_dynamic_state()
        self.pos_cmd = None
        self.last_open_force_send_s = 0.0
        self._reset_send_cache()
        rospy.loginfo("Enter OPEN_TO_START. reason=%s", reason)

    def _enter_wait_reapproach(self, now_s, snapshot):
        self._set_state("WAIT_REAPPROACH")
        self.wait_reapproach_enter_time_s = now_s
        self.release_intent_latched = False
        self.release_intent_until_s = 0.0
        self.target_trend_detector.reset()
        self.contact_lost_timer.reset()
        self.target_low_timer.reset()
        self.measured_low_timer.reset()
        self.last_hold_open_send_s = 0.0
        self._reset_send_cache()
        rospy.loginfo("Enter WAIT_REAPPROACH. Press 's' + Enter to APPROACH.")

    def _update_low_force_debug(self, now_s, snapshot):
        target_release = snapshot["target_abs_raw_N"]
        meas_release = snapshot["meas_force_f_N"]
        self.target_low_now = (
            snapshot["target_valid"] and
            target_release is not None and
            target_release <= self.release_target_threshold_N
        )
        self.measured_low_now = (
            snapshot["meas_valid"] and
            meas_release is not None and
            meas_release <= self.release_measured_threshold_N
        )
        self.target_low_confirmed = self.target_low_timer.update(
            self.target_low_now, now_s, self.release_target_low_confirm_s)
        self.measured_low_confirmed = self.measured_low_timer.update(
            self.measured_low_now, now_s, self.release_measured_low_confirm_s)
        self.low_force_ok = self.target_low_confirmed and self.measured_low_confirmed
        return self.low_force_ok

    def _trend_shape_guard_allows(self, metrics):
        if not metrics or self.trend_shape_guard_mode == "off":
            self.trend_shape_warning = False
            return True

        warnings = []
        if self.trend_use_end_cap and metrics.get("end_level_N", 0.0) > self.trend_max_end_N:
            warnings.append("end_level")
        if self.trend_use_min_drop_frac_high_force and \
           metrics.get("start_level_N", 0.0) >= self.trend_high_force_start_N and \
           metrics.get("drop_frac", 0.0) < self.trend_min_drop_frac_high_force:
            warnings.append("high_force_drop_frac")

        self.trend_shape_warning = bool(warnings)
        if warnings:
            rospy.logwarn_throttle(
                1.0,
                "Trend shape guard warning: %s metrics=%s",
                ",".join(warnings),
                metrics,
            )
        return not warnings if self.trend_shape_guard_mode == "enforce" else True

    def _publish_debug(self, now_s, snapshot):
        if self.debug_period_s <= 0.0 or (rospy.Time.now() - self._last_debug_t).to_sec() < self.debug_period_s:
            return

        m = self.last_trend_metrics or {}
        data = (
            f"state={self.state} prev_state={self.prev_state} "
            f"open_reason={self.open_reason} open_failed={self.open_failed} opened_enough={self.opened_enough} "
            f"target_raw_signed={snapshot['target_raw_signed_N'] if snapshot['target_raw_signed_N'] is not None else 'nan'} "
            f"target_abs_raw={snapshot['target_abs_raw_N'] if snapshot['target_abs_raw_N'] is not None else 'nan'} "
            f"target_f={snapshot['target_force_f_N'] if snapshot['target_force_f_N'] is not None else 'nan'} "
            f"target_age={snapshot['target_age_s']:.3f} target_valid={snapshot['target_valid']} "
            f"meas_raw_signed={snapshot['meas_raw_signed_N'] if snapshot['meas_raw_signed_N'] is not None else 'nan'} "
            f"meas_f={snapshot['meas_force_f_N'] if snapshot['meas_force_f_N'] is not None else 'nan'} "
            f"meas_age={snapshot['meas_age_s']:.3f} meas_valid={snapshot['meas_valid']} "
            f"width={snapshot['width_mm'] if snapshot['width_mm'] is not None else 'nan'} "
            f"width_age={snapshot['width_age_s']:.3f} width_valid={snapshot['width_valid']} "
            f"trend_n={m.get('n_samples', 0)} trend_drop={m.get('drop_N', float('nan'))} "
            f"trend_frac={m.get('drop_frac', float('nan'))} trend_slope={m.get('slope_N_per_s', float('nan'))} "
            f"trend_neg={m.get('neg_mag_ratio', float('nan'))} trend_eff={m.get('efficiency', float('nan'))} "
            f"trend_falling={m.get('falling', False)} trend_confirmed={m.get('confirmed', False)} "
            f"release_intent={self.release_intent_latched} release_triggered={self.release_open_triggered} "
            f"target_low={self.target_low_confirmed} meas_low={self.measured_low_confirmed} low_force_ok={self.low_force_ok} "
            f"contact_lost_now={self.contact_lost_now} contact_lost_confirmed={self.contact_lost_confirmed} "
            f"ignored_s_count={self.ignored_s_count}"
        )
        self.pub_debug.publish(String(data=data))
        self._last_debug_t = rospy.Time.now()

    # ===== 一个 tick（30Hz） =====
    def _tick(self, dt):
        self._check_target_timeout()
        now = rospy.Time.now()
        now_s = now.to_sec()
        snapshot = self._make_snapshot(now_s)

        requested = self._take_reapproach_request()
        if requested:
            if self.state in ("INIT", "WAIT_REAPPROACH"):
                if self.state == "WAIT_REAPPROACH" and self.warn_on_low_target_reapproach and snapshot["target_valid"] and \
                   snapshot["target_abs_raw_N"] < self.low_target_reapproach_warn_N:
                    rospy.logwarn(
                        "Re-approach accepted, but target force is low: %.3f N",
                        snapshot["target_abs_raw_N"],
                    )
                self._enter_approach(now_s, snapshot)
                self._publish_debug(now_s, snapshot)
                return
            self._record_ignored_s()

        # 状态逻辑
        if self.state == "INIT":
            self._publish_debug(now_s, snapshot)
            return

        elif self.state == "APPROACH":
            if not (snapshot["width_valid"] and snapshot["meas_valid"]):
                rospy.logwarn_throttle(
                    1.0,
                    "Skip APPROACH: width_valid=%s meas_valid=%s",
                    snapshot["width_valid"],
                    snapshot["meas_valid"],
                )
                self._publish_debug(now_s, snapshot)
                return

            if self.pos_cmd is None:
                base = snapshot["width_mm"] if snapshot["width_valid"] else self.start_width_mm
                self.pos_cmd = clamp(base, self.min_width_mm, self.max_width_mm)

            step = self.approach_speed_mm_s / self.rate_hz
            self.pos_cmd = clamp(self.pos_cmd - step, self.min_width_mm, self.max_width_mm)
            self._send_goal(self.pos_cmd, self.approach_speed_mm_s)

            if snapshot["meas_force_f_N"] is not None and snapshot["meas_force_f_N"] >= self.force_threshold_N:
                self._enter_force(now_s, snapshot)
                self._publish_debug(now_s, snapshot)
                return

        elif self.state == "FORCE":
            falling_confirmed = False
            trend_metrics = None
            if self.trend_open_enable and snapshot["target_valid"]:
                new_target_sample = (
                    self.last_trend_target_stamp_s is None or
                    snapshot["target_stamp_s"] > self.last_trend_target_stamp_s
                )
                if new_target_sample:
                    self.last_trend_target_stamp_s = snapshot["target_stamp_s"]
                    falling_confirmed, trend_metrics = self.target_trend_detector.update_and_check(
                        snapshot["target_stamp_s"],
                        snapshot["target_abs_raw_N"],
                    )
                    self.last_trend_metrics = trend_metrics
                    if falling_confirmed:
                        self.release_intent_latched = True
                        self.release_intent_until_s = now_s + self.release_intent_timeout_s
                        self.release_intent_reason = "target_falling"
                        self.release_trigger_metrics = dict(trend_metrics or {})

            self._update_low_force_debug(now_s, snapshot)

            release_intent_ok = (
                self.release_intent_latched and
                now_s <= self.release_intent_until_s
            )
            trend_release_valid = snapshot["target_valid"]
            if self.trend_release_require_meas_valid:
                trend_release_valid = trend_release_valid and snapshot["meas_valid"]
            if self.trend_release_require_status_valid:
                trend_release_valid = trend_release_valid and snapshot["width_valid"]

            if self.release_gate_mode == "trend_only" and trend_release_valid and release_intent_ok:
                if self._trend_shape_guard_allows(self.release_trigger_metrics):
                    self.release_open_triggered = True
                    self._enter_open_to_start(
                        now_s,
                        reason="target_falling",
                        snapshot=snapshot,
                        trend_metrics=self.release_trigger_metrics,
                    )
                    self._publish_debug(now_s, snapshot)
                    return

            if self.release_gate_mode == "trend_and_target_low" and trend_release_valid and release_intent_ok \
               and self.target_low_confirmed:
                self.release_open_triggered = True
                self._enter_open_to_start(now_s, reason="falling_and_target_low", snapshot=snapshot,
                                          trend_metrics=self.release_trigger_metrics)
                self._publish_debug(now_s, snapshot)
                return

            if self.release_gate_mode == "trend_and_low_force" and trend_release_valid and release_intent_ok \
               and self.low_force_ok:
                self.release_open_triggered = True
                self._enter_open_to_start(now_s, reason="falling_and_low_force", snapshot=snapshot,
                                          trend_metrics=self.release_trigger_metrics)
                self._publish_debug(now_s, snapshot)
                return

            if self.release_gate_mode == "trend_and_release_shape" and trend_release_valid and release_intent_ok:
                if self._trend_shape_guard_allows(self.release_trigger_metrics):
                    self.release_open_triggered = True
                    self._enter_open_to_start(now_s, reason="target_falling_shape", snapshot=snapshot,
                                              trend_metrics=self.release_trigger_metrics)
                    self._publish_debug(now_s, snapshot)
                    return

            self.contact_lost_now = False
            self.contact_lost_confirmed = False
            self.contact_lost_in_grace = False
            if self.force_contact_lost_to_open_enable and snapshot["meas_valid"]:
                self.contact_lost_in_grace = (
                    self.force_enter_time_s is not None and
                    (now_s - self.force_enter_time_s) < self.force_contact_lost_grace_s
                )
                if not self.contact_lost_in_grace:
                    self.contact_lost_now = snapshot["meas_force_f_N"] < self.force_contact_lost_threshold_N
                    self.contact_lost_confirmed = self.contact_lost_timer.update(
                        self.contact_lost_now,
                        now_s,
                        self.force_contact_lost_confirm_s,
                    )
                    if self.contact_lost_confirmed:
                        self.contact_lost_open_triggered = True
                        self._enter_open_to_start(now_s, reason="contact_lost", snapshot=snapshot)
                        self._publish_debug(now_s, snapshot)
                        return
                else:
                    self.contact_lost_timer.reset()

            if not (snapshot["target_valid"] and snapshot["meas_valid"] and snapshot["width_valid"]):
                if self.reset_pid_on_stale:
                    self._reset_pid_dynamic_state()
                rospy.logwarn_throttle(
                    1.0,
                    "Skip FORCE PID: target_valid=%s meas_valid=%s width_valid=%s",
                    snapshot["target_valid"],
                    snapshot["meas_valid"],
                    snapshot["width_valid"],
                )
                self._publish_debug(now_s, snapshot)
                return

            if snapshot["target_force_f_N"] is None or snapshot["meas_force_f_N"] is None:
                self._publish_debug(now_s, snapshot)
                return

            # 控制用目标力：回调中已 abs + 滤波；这里做缩放与死区
            raw_target = snapshot["target_abs_raw_N"] if snapshot["target_abs_raw_N"] is not None else float("nan")
            scaled_target = self.target_scale * snapshot["target_force_f_N"]
            tgt = 0.0 if abs(scaled_target) <= self.target_deadband_N else scaled_target
            # PID（误差: 缩放后目标力 - 实测力）
            err = tgt - snapshot["meas_force_f_N"]

            # 积分启用门限 & 反风up限幅（以位移单位限制积分项）
            if abs(err) <= self.i_enable_band_N:
                self.int_acc += err * dt
            i_term = self.ki * self.int_acc
            if i_term > self.i_limit_mm:  i_term = self.i_limit_mm
            if i_term < -self.i_limit_mm: i_term = -self.i_limit_mm

            d_term = 0.0 if self.prev_err is None else self.kd * (err - self.prev_err) / max(dt, 1e-6)
            self.prev_err = err
            self.prev_pid_time_s = now_s

            delta_mm = self.kp*err + i_term + d_term
            new_width = clamp(snapshot["width_mm"] - delta_mm, self.min_width_mm, self.max_width_mm)
            print("FORCE CTRL: target_raw=%.3f N, target_ctrl=%.3f N, meas=%.3f N, err=%.3f N ,new_width=%.2f mm"
                  % (raw_target, tgt, snapshot["meas_force_f_N"], err, new_width))
            self._send_goal(new_width, self.pid_speed_mm_s)

            # ---- 新增：FORCE 期间记录（相对 FORCE 起点时间）----
            if self._t_force_start is not None and snapshot["meas_force_f_N"] is not None:
                t_rel = (now - self._t_force_start).to_sec()
                self._force_log.append({
                    "ros_time_s": now.to_sec(),
                    "t_rel_s": t_rel,
                    "target_raw_N": raw_target,
                    "target_filtered_N": snapshot["target_force_f_N"] if snapshot["target_force_f_N"] is not None else float("nan"),
                    "target_ctrl_N": float(tgt),
                    "measured_raw_N": snapshot["meas_scaled_nonneg_N"] if snapshot["meas_scaled_nonneg_N"] is not None else float("nan"),
                    "measured_filtered_N": float(snapshot["meas_force_f_N"]),
                    "error_N": float(err),
                    "width_mm": snapshot["width_mm"] if snapshot["width_mm"] is not None else float("nan"),
                    "cmd_width_mm": float(new_width),
                })

        elif self.state == "OPEN_TO_START":
            force_resend = (now_s - self.last_open_force_send_s) >= self.open_command_force_resend_period_s
            sent = self._send_goal(self.start_width_mm, self.open_speed_mm_s, force=force_resend)
            if force_resend and sent:
                self.last_open_force_send_s = now_s

            self.opened_enough = (
                snapshot["width_valid"] and
                snapshot["width_mm"] >= self.start_width_mm - self.open_width_tol_mm
            )
            min_hold_done = (
                self.open_enter_time_s is not None and
                now_s - self.open_enter_time_s >= self.open_min_hold_s
            )
            timeout = (
                self.open_enter_time_s is not None and
                now_s - self.open_enter_time_s >= self.open_timeout_s
            )
            if (self.opened_enough and min_hold_done) or timeout:
                self.open_failed = timeout and not self.opened_enough
                if self.open_failed:
                    rospy.logwarn("OPEN_TO_START timeout before reaching start_width.")
                self._enter_wait_reapproach(now_s, snapshot)
                self._publish_debug(now_s, snapshot)
                return

        elif self.state == "WAIT_REAPPROACH":
            force_resend = (now_s - self.last_hold_open_send_s) >= self.hold_open_command_period_s
            sent = self._send_goal(self.start_width_mm, self.hold_open_speed_mm_s, force=force_resend)
            if force_resend and sent:
                self.last_hold_open_send_s = now_s

        self._publish_debug(now_s, snapshot)

    def _make_timestamped_csv_path(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        if not self.csv_path:
            return os.path.join(os.getcwd(), f"wsg50_force_ctrl_{timestamp}.csv")

        path = os.path.expanduser(str(self.csv_path))
        if path.endswith(os.sep) or os.path.isdir(path):
            return os.path.join(path, f"wsg50_force_ctrl_{timestamp}.csv")

        root, ext = os.path.splitext(path)
        if not ext:
            return os.path.join(path, f"wsg50_force_ctrl_{timestamp}.csv")
        if ext.lower() == ".csv":
            return f"{root}_{timestamp}.csv"
        return f"{path}_{timestamp}.csv"

    def _save_force_csv(self):
        if self._csv_saved or not self.save_csv or not self._force_log:
            return

        path = self._make_timestamped_csv_path()
        out_dir = os.path.dirname(path)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir)

        fieldnames = [
            "ros_time_s",
            "t_rel_s",
            "target_raw_N",
            "target_filtered_N",
            "target_ctrl_N",
            "measured_raw_N",
            "measured_filtered_N",
            "error_N",
            "width_mm",
            "cmd_width_mm",
        ]
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._force_log)

        self._csv_saved = True
        rospy.loginfo("Force control CSV saved to: %s", path)

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

            self._save_force_csv()

            # 解包记录：ts[0..N-1], meas_abs[k], tgt_abs[k]
            ts = tuple(row["t_rel_s"] for row in self._force_log)
            meas_abs = tuple(abs(row["measured_filtered_N"]) for row in self._force_log)
            tgt_abs = tuple(abs(row["target_ctrl_N"]) for row in self._force_log)
            n = len(meas_abs)

            # ---- 上升时间：相对稳态值 Yss 的 ~self.rise_frac ----
            Yss = meas_abs[-1]
            if Yss <= 1e-9:
                rospy.loginfo(
                    "Steady-state force ~0; cannot compute times. PID[Kp=%.3f, Ki=%.3f, Kd=%.3f]",
                    getattr(self, "kp", float("nan")), getattr(self, "ki", float("nan")), getattr(self, "kd", float("nan"))
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
