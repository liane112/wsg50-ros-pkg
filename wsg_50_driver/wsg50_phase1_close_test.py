#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase-1 低速闭合（110 -> 0）测试（auto_update 30Hz 跟随）
- 一旦 F >= 阈值：将位置指令“钉死”在触发当下的当前位置（不再继续逼近 0 mm）
- 若先到达最小行程：停在 min_width（防硬顶）
"""

import rospy
from rospy import AnyMsg
from roslib.message import get_message_class
from wsg_50_common.msg import Cmd, Status

def clamp(x, lo, hi): return max(lo, min(hi, x))

def extract_scalar_from_msg(msg, index=0):
    if hasattr(msg, "data") and isinstance(msg.data, (float, int)):
        return float(msg.data)
    if hasattr(msg, "data") and hasattr(msg.data, "__len__") and len(msg.data) > 0:
        idx = index if 0 <= index < len(msg.data) else 0
        return float(msg.data[idx])
    return None

class Phase1CloseTest(object):
    def __init__(self):
        rospy.init_node("wsg50_phase1_close_test_v2_1")

        # 参数
        self.goal_position_topic = rospy.get_param("~goal_position_topic", "/wsg_50_driver/goal_position")
        self.status_topic        = rospy.get_param("~status_topic",        "/wsg_50_driver/status")
        self.force_topic         = rospy.get_param("~force_topic",         "/znsv6_data_sensor1")

        self.force_index     = int(rospy.get_param("~force_index", 0))
        self.stop_force_N    = float(rospy.get_param("~stop_force_N", 0.1))
        self.force_alpha     = float(rospy.get_param("~force_lpf_alpha", 0.3))

        self.cmd_rate_hz     = float(rospy.get_param("~cmd_rate_hz", 30.0))
        self.close_speed_mm_s= float(rospy.get_param("~close_speed_mm_s", 20.0))
        self.hold_speed_mm_s = float(rospy.get_param("~hold_speed_mm_s", 10.0))  # WSG-50 最小速度建议 ≥5

        self.min_width_mm    = float(rospy.get_param("~min_width_mm", 0.0))
        self.max_width_mm    = float(rospy.get_param("~max_width_mm", 110.0))
        self.min_speed_mm_s  = float(rospy.get_param("~min_speed_mm_s", 5.0))
        self.max_speed_mm_s  = float(rospy.get_param("~max_speed_mm_s", 420.0))

        self.start_width_mm  = float(rospy.get_param("~start_width_mm", 110.0))

        # 运行态
        self.width_meas_mm = None
        self.force_meas_f  = None
        self._force_msg_cls = None

        self.pos_cmd = None
        self.pos_cmd_inited = False
        self.stop_flag = False
        self.stop_reason = None      # "force" or "minwidth"
        self.hold_pos_mm = None      # ★ 力触发时锁定的“保持位置”

        # 通信
        self.cmd_pub = rospy.Publisher(self.goal_position_topic, Cmd, queue_size=50)
        rospy.Subscriber(self.status_topic, Status, self._status_cb, queue_size=50)
        rospy.Subscriber(self.force_topic, AnyMsg, self._force_cb_any, queue_size=100)

        rospy.loginfo("Phase-1 v2.1 started: cmd@%.1f Hz, close=%.1f mm/s, stopF=%.3f N",
                      self.cmd_rate_hz, self.close_speed_mm_s, self.stop_force_N)

    def _status_cb(self, msg: Status):
        self.width_meas_mm = float(msg.width)  # mm

    def _force_cb_any(self, any_msg: AnyMsg):
        try:
            if self._force_msg_cls is None:
                typ = any_msg._connection_header.get('type', '')
                self._force_msg_cls = get_message_class(typ)
                rospy.loginfo("Detected force msg type: %s", typ)
            m = self._force_msg_cls(); m.deserialize(any_msg._buff)
            v = extract_scalar_from_msg(m, self.force_index)
            if v is None: return
            f = float(v)
            if self.force_meas_f is None:
                self.force_meas_f = f
            else:
                a = clamp(self.force_alpha, 0.0, 1.0)
                self.force_meas_f = a*f + (1.0-a)*self.force_meas_f
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Force parse failed: %s", e)

    def _send_cmd(self, pos_mm, speed_mm_s):
        cmd = Cmd()
        cmd.pos   = clamp(pos_mm, self.min_width_mm, self.max_width_mm)
        cmd.speed = clamp(abs(speed_mm_s), self.min_speed_mm_s, self.max_speed_mm_s)
        self.cmd_pub.publish(cmd)

    def run(self):
        rate = rospy.Rate(self.cmd_rate_hz)
        step_mm = max(1e-9, self.close_speed_mm_s / self.cmd_rate_hz)
        last_log_t = rospy.Time.now()

        while not rospy.is_shutdown():
            # 初始化命令位置
            if not self.pos_cmd_inited:
                self.pos_cmd = self.width_meas_mm if self.width_meas_mm is not None else self.start_width_mm
                self.pos_cmd = clamp(self.pos_cmd, self.min_width_mm, self.max_width_mm)
                self.pos_cmd_inited = True
                rospy.loginfo("Init pos_cmd = %.2f mm", self.pos_cmd)

            f = self.force_meas_f if self.force_meas_f is not None else 0.0

            if not self.stop_flag:
                # ★ 1) 力触发：锁定“保持位置”为当前实测宽度（无论 pos_cmd 是多少）
                if f >= self.stop_force_N:
                    self.stop_flag = True
                    self.stop_reason = "force"
                    self.hold_pos_mm = clamp(self.width_meas_mm if self.width_meas_mm is not None else self.pos_cmd,
                                             self.min_width_mm, self.max_width_mm)
                    rospy.loginfo("STOP by force: F=%.3f N, HOLD @ %.2f mm", f, self.hold_pos_mm)
                    # 立即发一次保持
                    self._send_cmd(self.hold_pos_mm, self.hold_speed_mm_s)

                else:
                    # 2) 未触发则继续闭合；若先到达最小行程则以 minwidth 停止
                    self.pos_cmd = clamp(self.pos_cmd - step_mm, self.min_width_mm, self.max_width_mm)
                    if self.pos_cmd <= self.min_width_mm + 1e-6:
                        self.stop_flag = True
                        self.stop_reason = "minwidth"
                        self.hold_pos_mm = self.min_width_mm
                        rospy.logwarn("Reached min width (%.2f mm) before force threshold. HOLD @ %.2f mm",
                                      self.pos_cmd, self.hold_pos_mm)
                    self._send_cmd(self.pos_cmd, self.close_speed_mm_s)

            else:
                # 已停止：力触发 → 永远按“触发当刻”的 hold_pos_mm 发保持
                if self.stop_reason == "force" and self.hold_pos_mm is not None:
                    self._send_cmd(self.hold_pos_mm, self.hold_speed_mm_s)
                else:
                    # 撞到底的情况：保持在 min_width
                    self._send_cmd(self.hold_pos_mm if self.hold_pos_mm is not None else self.min_width_mm,
                                   self.hold_speed_mm_s)

            # 节流日志
            now = rospy.Time.now()
            if (now - last_log_t).to_sec() > 0.5:
                hold = (self.hold_pos_mm if (self.stop_flag and self.hold_pos_mm is not None) else self.pos_cmd)
                rospy.loginfo("cmd pos=%.2f mm, speed=%.1f mm/s, F=%.3f N, stop=%s, reason=%s",
                              hold,
                              (self.hold_speed_mm_s if self.stop_flag else self.close_speed_mm_s),
                              f, self.stop_flag, (self.stop_reason or "-"))
                last_log_t = now
            rate.sleep()

if __name__ == "__main__":
    try:
        Phase1CloseTest().run()
    except rospy.ROSInterruptException:
        pass

