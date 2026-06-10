#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线仿真测试 wsg50_fsm_force_ctrl.py 的状态切换。

默认行为：
- 自动启动 wsg50_fsm_force_ctrl.py 子进程；
- 往子进程 stdin 写入 "s"，模拟人工启动 APPROACH；
- 发布虚拟 /wsg_50_driver/status、/znsv6_data_sensor2、/znsv6_cmd/act2；
- 订阅 /wsg_50_driver/goal_position，让虚拟夹爪跟随 FSM 命令；
- 观察 debug topic，检查状态是否按 APPROACH -> FORCE -> OPEN_TO_START -> WAIT_REAPPROACH 切换。

也可以设置 _start_fsm:=false，只作为假设备/假力信号发布器使用。
"""

import os
import re
import signal
import subprocess
import sys
import time

import rospy
from std_msgs.msg import Float32, String
from wsg_50_common.msg import Cmd, Status


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class WSG50FSMSimTest(object):
    def __init__(self):
        rospy.init_node("wsg50_fsm_sim_test", anonymous=True)

        self.start_fsm = bool(rospy.get_param("~start_fsm", True))
        self.auto_send_s = bool(rospy.get_param("~auto_send_s", True))
        self.timeout_s = float(rospy.get_param("~timeout_s", 14.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        self.goal_topic = rospy.get_param("~goal_position_topic", "/wsg_50_driver/goal_position")
        self.status_topic = rospy.get_param("~status_topic", "/wsg_50_driver/status")
        self.measured_topic = rospy.get_param("~measured_force_topic", "/znsv6_data_sensor2")
        self.target_topic = rospy.get_param("~target_force_topic", "/znsv6_cmd/act2")
        self.debug_topic = rospy.get_param("~debug_topic", "/wsg50_fsm_force_ctrl/debug")

        self.start_width_mm = float(rospy.get_param("~start_width_mm", 110.0))
        self.min_width_mm = float(rospy.get_param("~min_width_mm", 0.0))
        self.max_width_mm = float(rospy.get_param("~max_width_mm", 110.0))
        self.object_width_mm = float(rospy.get_param("~object_width_mm", 64.0))
        self.object_stiffness_N_per_mm = float(rospy.get_param("~object_stiffness_N_per_mm", 1.4))
        self.force_lpf_alpha = float(rospy.get_param("~sim_force_lpf_alpha", 0.35))

        self.target_high_N = float(rospy.get_param("~target_high_N", 4.0))
        self.target_low_N = float(rospy.get_param("~target_low_N", 3.1))
        self.force_hold_before_drop_s = float(rospy.get_param("~force_hold_before_drop_s", 1.2))
        self.target_drop_s = float(rospy.get_param("~target_drop_s", 0.45))

        self.width_mm = self.start_width_mm
        self.cmd_pos_mm = self.start_width_mm
        self.cmd_speed_mm_s = 50.0
        self.measured_force_N = 0.0
        self.target_force_N = self.target_high_N

        self.state = None
        self.open_reason = ""
        self.last_debug = ""
        self.transitions = []
        self.force_enter_wall = None
        self.drop_start_wall = None
        self.wait_enter_wall = None
        self.fsm_proc = None

        self.pub_status = rospy.Publisher(self.status_topic, Status, queue_size=20)
        self.pub_measured = rospy.Publisher(self.measured_topic, Float32, queue_size=20)
        self.pub_target = rospy.Publisher(self.target_topic, Float32, queue_size=20)
        rospy.Subscriber(self.goal_topic, Cmd, self._cmd_cb, queue_size=20)
        rospy.Subscriber(self.debug_topic, String, self._debug_cb, queue_size=50)

    def _cmd_cb(self, msg):
        self.cmd_pos_mm = clamp(float(msg.pos), self.min_width_mm, self.max_width_mm)
        self.cmd_speed_mm_s = max(1e-6, abs(float(msg.speed)))

    def _debug_cb(self, msg):
        self.last_debug = msg.data
        state_match = re.search(r"\bstate=([A-Z_]+)", msg.data)
        reason_match = re.search(r"\bopen_reason=([^ ]*)", msg.data)
        if reason_match:
            self.open_reason = reason_match.group(1)
        if not state_match:
            return

        new_state = state_match.group(1)
        if new_state != self.state:
            now = time.time()
            self.transitions.append((now, new_state, self.open_reason))
            rospy.loginfo("FSM state: %s -> %s reason=%s", self.state, new_state, self.open_reason)
            self.state = new_state
            if new_state == "FORCE" and self.force_enter_wall is None:
                self.force_enter_wall = now
            if new_state == "WAIT_REAPPROACH" and self.wait_enter_wall is None:
                self.wait_enter_wall = now

    def _start_fsm_proc(self):
        if not self.start_fsm:
            return

        script_dir = os.path.dirname(os.path.abspath(__file__))
        fsm_path = os.path.join(script_dir, "wsg50_fsm_force_ctrl.py")
        args = [
            sys.executable,
            fsm_path,
            "_debug_topic:={}".format(self.debug_topic),
            "_debug_period_s:=0.05",
            "_save_csv:=False",
            "_save_plot:=False",
            "_target_timeout_s:=2.0",
            "_target_stale_timeout_s:=0.60",
            "_meas_stale_timeout_s:=0.60",
            "_status_stale_timeout_s:=0.60",
            "_release_gate_mode:=trend_only",
            "_trend_min_samples:=10",
            "_trend_min_window_s:=0.35",
            "_trend_window_s:=0.60",
        ]
        self.fsm_proc = subprocess.Popen(
            args,
            stdin=subprocess.PIPE,
            stdout=None,
            stderr=None,
            preexec_fn=os.setsid,
            universal_newlines=True,
        )
        rospy.loginfo("Started FSM subprocess pid=%s", self.fsm_proc.pid)

    def _stop_fsm_proc(self):
        if self.fsm_proc is None:
            return
        if self.fsm_proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.fsm_proc.pid), signal.SIGINT)
            self.fsm_proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(self.fsm_proc.pid), signal.SIGKILL)
            except Exception:
                pass

    def _send_start_key(self):
        if not (self.auto_send_s and self.fsm_proc and self.fsm_proc.stdin):
            return
        try:
            self.fsm_proc.stdin.write("s\n")
            self.fsm_proc.stdin.flush()
            rospy.loginfo("Sent 's' to FSM stdin.")
        except Exception as exc:
            rospy.logwarn("Failed to send 's' to FSM stdin: %s", exc)

    def _update_virtual_width(self, dt):
        diff = self.cmd_pos_mm - self.width_mm
        max_step = self.cmd_speed_mm_s * dt
        if abs(diff) <= max_step:
            self.width_mm = self.cmd_pos_mm
        else:
            self.width_mm += max_step if diff > 0.0 else -max_step
        self.width_mm = clamp(self.width_mm, self.min_width_mm, self.max_width_mm)

    def _update_virtual_force(self):
        compression_mm = max(0.0, self.object_width_mm - self.width_mm)
        raw_force = compression_mm * self.object_stiffness_N_per_mm
        raw_force = clamp(raw_force, 0.0, 8.0)
        a = clamp(self.force_lpf_alpha, 0.0, 1.0)
        self.measured_force_N = a * raw_force + (1.0 - a) * self.measured_force_N

    def _update_target_force(self):
        now = time.time()
        if self.force_enter_wall is None:
            self.target_force_N = self.target_high_N
            return

        if self.drop_start_wall is None:
            if now - self.force_enter_wall >= self.force_hold_before_drop_s:
                self.drop_start_wall = now
                rospy.loginfo(
                    "Start target drop: %.3f N -> %.3f N in %.3f s",
                    self.target_high_N,
                    self.target_low_N,
                    self.target_drop_s,
                )
            else:
                self.target_force_N = self.target_high_N
                return

        p = clamp((now - self.drop_start_wall) / max(self.target_drop_s, 1e-6), 0.0, 1.0)
        self.target_force_N = self.target_high_N + p * (self.target_low_N - self.target_high_N)

    def _publish_fake_io(self):
        st = Status()
        st.status = "sim"
        st.width = self.width_mm
        st.speed = self.cmd_speed_mm_s
        st.acc = 0.0
        st.force = self.measured_force_N
        st.force_finger0 = 0.5 * self.measured_force_N
        st.force_finger1 = 0.5 * self.measured_force_N
        self.pub_status.publish(st)
        self.pub_measured.publish(Float32(data=self.measured_force_N))
        self.pub_target.publish(Float32(data=self.target_force_N))

    def _has_required_transitions(self):
        required = ["APPROACH", "FORCE", "OPEN_TO_START", "WAIT_REAPPROACH"]
        idx = 0
        for _, state, _ in self.transitions:
            if idx < len(required) and state == required[idx]:
                idx += 1
        return idx == len(required)

    def run(self):
        self._start_fsm_proc()
        rospy.sleep(0.8)

        start_wall = time.time()
        sent_s = False
        rate = rospy.Rate(self.rate_hz)
        last_wall = time.time()

        try:
            while not rospy.is_shutdown():
                now = time.time()
                dt = max(1.0 / self.rate_hz, now - last_wall)
                last_wall = now

                self._update_target_force()
                self._update_virtual_width(dt)
                self._update_virtual_force()
                self._publish_fake_io()

                if not sent_s and now - start_wall >= 0.6:
                    self._send_start_key()
                    sent_s = True

                if self.wait_enter_wall is not None and now - self.wait_enter_wall >= 0.4:
                    break
                if now - start_wall >= self.timeout_s:
                    break
                rate.sleep()
        finally:
            self._stop_fsm_proc()

        ok = self._has_required_transitions()
        reason_ok = self.open_reason == "target_falling"
        rospy.loginfo("Transitions: %s", " -> ".join(s for _, s, _ in self.transitions))
        rospy.loginfo("Final state=%s open_reason=%s width=%.2f target=%.3f meas=%.3f",
                      self.state, self.open_reason, self.width_mm, self.target_force_N, self.measured_force_N)
        if ok and reason_ok:
            rospy.loginfo("SIM TEST PASS: state machine reached WAIT_REAPPROACH by target_falling.")
            return 0

        rospy.logerr("SIM TEST FAIL: required transitions or open_reason not observed.")
        rospy.logerr("Last debug: %s", self.last_debug)
        return 1


if __name__ == "__main__":
    try:
        sys.exit(WSG50FSMSimTest().run())
    except rospy.ROSInterruptException:
        pass
