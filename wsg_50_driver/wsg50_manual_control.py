#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Interactive position control for the WSG-50 auto_update driver mode."""

import math
import shlex
import sys
import termios
import threading
import tty

import rospy
from wsg_50_common.msg import Cmd, Status


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class WSG50ManualControl(object):
    def __init__(self):
        rospy.init_node("wsg50_manual_control", anonymous=False)

        self.goal_topic = rospy.get_param(
            "~goal_position_topic", "/wsg_50_driver/goal_position")
        self.status_topic = rospy.get_param(
            "~status_topic", "/wsg_50_driver/status")
        self.driver_mode_param = rospy.get_param(
            "~driver_mode_param", "/wsg_50_driver/com_mode")

        self.min_width_mm = float(rospy.get_param("~min_width_mm", 1.0))
        self.max_width_mm = float(rospy.get_param("~max_width_mm", 109.0))
        self.min_speed_mm_s = float(rospy.get_param("~min_speed_mm_s", 1.0))
        self.max_speed_mm_s = float(rospy.get_param("~max_speed_mm_s", 50.0))
        self.speed_mm_s = float(rospy.get_param("~speed_mm_s", 10.0))
        self.fine_step_mm = float(rospy.get_param("~fine_step_mm", 0.2))
        self.normal_step_mm = float(rospy.get_param("~normal_step_mm", 1.0))
        self.coarse_step_mm = float(rospy.get_param("~coarse_step_mm", 5.0))
        self.status_timeout_s = float(rospy.get_param("~status_timeout_s", 0.5))
        self.startup_timeout_s = float(rospy.get_param("~startup_timeout_s", 5.0))
        self.hold_speed_mm_s = float(rospy.get_param("~hold_speed_mm_s", 1.0))
        self.exit_hold_wait_s = float(rospy.get_param("~exit_hold_wait_s", 0.1))
        self.publisher_check_period_s = float(
            rospy.get_param("~publisher_check_period_s", 1.0))
        self.allow_shared_publisher = bool(
            rospy.get_param("~allow_shared_publisher", False))

        if self.min_width_mm >= self.max_width_mm:
            raise ValueError("min_width_mm must be smaller than max_width_mm")
        if self.min_speed_mm_s <= 0.0 or self.min_speed_mm_s > self.max_speed_mm_s:
            raise ValueError("invalid manual speed range")
        if min(self.fine_step_mm, self.normal_step_mm, self.coarse_step_mm) <= 0.0:
            raise ValueError("manual increments must be positive")
        if self.status_timeout_s <= 0.0:
            raise ValueError("status_timeout_s must be positive")
        if not self.min_speed_mm_s <= self.hold_speed_mm_s <= self.max_speed_mm_s:
            raise ValueError("hold_speed_mm_s is outside the manual speed range")
        if self.exit_hold_wait_s < 0.0:
            raise ValueError("exit_hold_wait_s must not be negative")

        self.speed_mm_s = clamp(
            self.speed_mm_s, self.min_speed_mm_s, self.max_speed_mm_s)
        driver_mode = rospy.get_param(self.driver_mode_param, None)
        if driver_mode is None:
            rospy.logwarn(
                "Cannot verify driver mode: parameter %s is missing",
                self.driver_mode_param,
            )
        elif str(driver_mode).strip() != "auto_update":
            raise RuntimeError(
                "driver mode must be auto_update, got %r from %s" %
                (driver_mode, self.driver_mode_param))

        self._lock = threading.RLock()
        self._status = None
        self._status_stamp = None
        self._target_width_mm = None
        self._command_sent = False
        self._stale_hold_sent = False
        self._conflict_latched = False
        self._conflict_hold_sent = False
        self._shutdown_hold_sent = False
        self._last_publisher_check_s = 0.0

        self._status_sub = rospy.Subscriber(
            self.status_topic, Status, self._status_cb, queue_size=20)

        conflicts = self._other_goal_publishers()
        if conflicts and not self.allow_shared_publisher:
            raise RuntimeError(
                "goal topic already has publisher(s): %s" % ", ".join(conflicts))

        self._goal_pub = rospy.Publisher(
            self.goal_topic, Cmd, queue_size=10)

        self._safety_timer = rospy.Timer(
            rospy.Duration(0.1), self._safety_tick)
        rospy.on_shutdown(self._on_shutdown)

    def _status_cb(self, msg):
        with self._lock:
            self._status = msg
            self._status_stamp = rospy.Time.now()
            self._stale_hold_sent = False
            if self._target_width_mm is None and math.isfinite(float(msg.width)):
                self._target_width_mm = clamp(
                    float(msg.width), self.min_width_mm, self.max_width_mm)

    def _other_goal_publishers(self):
        try:
            response = rospy.get_master().getSystemState()
            if (isinstance(response, (list, tuple)) and
                    len(response) == 3 and isinstance(response[0], int)):
                code, message, state = response
                if code != 1:
                    raise rospy.ROSException(message)
            else:
                state = response
            resolved_topic = rospy.resolve_name(self.goal_topic)
            own_node = rospy.get_name()
            for topic, nodes in state[0]:
                if topic == resolved_topic:
                    return sorted(node for node in nodes if node != own_node)
        except Exception as exc:
            rospy.logwarn("Publisher conflict check failed: %s", exc)
        return []

    def _status_age_s(self):
        with self._lock:
            stamp = self._status_stamp
        if stamp is None:
            return float("inf")
        return max(0.0, (rospy.Time.now() - stamp).to_sec())

    def _status_is_fresh(self):
        return self._status_age_s() <= self.status_timeout_s

    def _hold_current_position(self, reason):
        with self._lock:
            status = self._status
        if status is None:
            rospy.logerr(
                "Cannot hold position for %s: no WSG status is available", reason)
            return False

        width_mm = float(status.width)
        if not math.isfinite(width_mm):
            rospy.logerr(
                "Cannot hold position for %s: invalid width %r",
                reason,
                status.width,
            )
            return False
        if self._goal_pub.get_num_connections() <= 0:
            rospy.logerr(
                "Cannot hold position for %s: WSG driver is not subscribed",
                reason,
            )
            return False

        age_s = self._status_age_s()
        if age_s > self.status_timeout_s:
            rospy.logwarn(
                "Holding the last known width after stale status: "
                "age=%.3f s limit=%.3f s",
                age_s,
                self.status_timeout_s,
            )

        target_width_mm = clamp(
            width_mm, self.min_width_mm, self.max_width_mm)
        msg = Cmd()
        msg.mode = "move"
        msg.pos = target_width_mm
        msg.speed = self.hold_speed_mm_s
        self._goal_pub.publish(msg)

        with self._lock:
            self._target_width_mm = target_width_mm
            self._command_sent = False

        rospy.logwarn(
            "Hold command: reason=%s target=%.3f mm speed=%.3f mm/s",
            reason,
            target_width_mm,
            self.hold_speed_mm_s,
        )
        return True

    def _safety_tick(self, _event):
        if rospy.is_shutdown():
            return

        now_s = rospy.Time.now().to_sec()
        if (now_s - self._last_publisher_check_s) >= \
                self.publisher_check_period_s:
            self._last_publisher_check_s = now_s
            conflicts = self._other_goal_publishers()
            if conflicts and not self.allow_shared_publisher:
                with self._lock:
                    self._conflict_latched = True
                    should_hold = not self._conflict_hold_sent
                    self._conflict_hold_sent = True
                if should_hold:
                    rospy.logerr(
                        "Goal publisher conflict detected: %s",
                        ", ".join(conflicts),
                    )
                    self._hold_current_position("publisher conflict")

        with self._lock:
            status_seen = self._status_stamp is not None
            should_hold_stale = (
                status_seen and
                self._command_sent and
                not self._stale_hold_sent and
                not self._status_is_fresh()
            )
            if should_hold_stale:
                self._stale_hold_sent = True

        if should_hold_stale:
            self._hold_current_position("status timeout")

    def _wait_until_ready(self):
        deadline = rospy.Time.now() + rospy.Duration(self.startup_timeout_s)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self._status_is_fresh() and self._goal_pub.get_num_connections() > 0:
                return True
            rate.sleep()

        rospy.logerr(
            "Manual control is not ready: status_fresh=%s goal_subscribers=%d",
            self._status_is_fresh(),
            self._goal_pub.get_num_connections(),
        )
        return False

    def _can_move(self):
        with self._lock:
            conflict = self._conflict_latched
        if conflict:
            rospy.logerr("Motion rejected: another goal publisher was detected")
            return False
        if not self._status_is_fresh():
            rospy.logerr(
                "Motion rejected: status age %.3f s exceeds %.3f s",
                self._status_age_s(),
                self.status_timeout_s,
            )
            return False
        if self._goal_pub.get_num_connections() <= 0:
            rospy.logerr("Motion rejected: WSG driver is not subscribed")
            return False
        return True

    def _publish_target(self, target_width_mm):
        if not self._can_move():
            return False

        target_width_mm = float(target_width_mm)
        if not self.min_width_mm <= target_width_mm <= self.max_width_mm:
            rospy.logerr(
                "Target %.3f mm is outside [%.3f, %.3f] mm",
                target_width_mm,
                self.min_width_mm,
                self.max_width_mm,
            )
            return False

        msg = Cmd()
        msg.mode = "move"
        msg.pos = target_width_mm
        msg.speed = self.speed_mm_s
        self._goal_pub.publish(msg)

        with self._lock:
            self._target_width_mm = target_width_mm
            self._command_sent = True

        rospy.loginfo(
            "Command: target=%.3f mm speed=%.3f mm/s",
            target_width_mm,
            self.speed_mm_s,
        )
        return True

    def _move_increment(self, delta_mm):
        if not self._can_move():
            return False
        with self._lock:
            base = self._target_width_mm
            if base is None and self._status is not None:
                base = float(self._status.width)
        if base is None:
            rospy.logerr("No valid width is available")
            return False

        requested = base + float(delta_mm)
        target = clamp(requested, self.min_width_mm, self.max_width_mm)
        if target != requested:
            rospy.logwarn(
                "Increment limited by software width range to %.3f mm", target)
        return self._publish_target(target)

    def _print_status(self):
        with self._lock:
            status = self._status
            target = self._target_width_mm
        if status is None:
            print("No status received")
            return
        print(
            "actual={:.3f} mm target={} speed={:.3f} mm/s "
            "motor_force={:.3f} N age={:.3f} s state={}".format(
                float(status.width),
                "{:.3f} mm".format(target) if target is not None else "n/a",
                float(status.speed),
                float(status.force),
                self._status_age_s(),
                status.status,
            )
        )

    def _print_help(self):
        print(
            "\n"
            "Immediate keys (no Enter):\n"
            "  o / p / i    open by {:.1f} / {:.1f} / {:.1f} mm\n"
            "  c / v / x    close by {:.1f} / {:.1f} / {:.1f} mm\n"
            "  s            hold the latest measured width\n"
            "  t            print current status\n"
            "  h or ?       show this help\n"
            "  q            hold the latest measured width and exit\n"
            "\n"
            "Prompt keys:\n"
            "  g            enter an absolute opening width in mm\n"
            "  r            enter a new movement speed in mm/s\n"
            "  :            enter a full command, then press Enter\n"
            "\n"
            "Full commands available after ':':\n"
            "  g <width>, goto <width>, open, speed <v>, stop,\n"
            "  home, status, help, q\n"
            .format(
                self.normal_step_mm,
                self.coarse_step_mm,
                self.fine_step_mm,
                self.normal_step_mm,
                self.coarse_step_mm,
                self.fine_step_mm,
            )
        )

    @staticmethod
    def _read_single_key():
        fd = sys.stdin.fileno()
        original_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSANOW, original_settings)

    @staticmethod
    def _parse_positive_arg(tokens, default_value=None):
        if len(tokens) == 1 and default_value is not None:
            return default_value
        if len(tokens) != 2:
            raise ValueError("expected one numeric argument")
        value = float(tokens[1])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("value must be a positive finite number")
        return value

    def _handle_command(self, line):
        tokens = shlex.split(line)
        if not tokens:
            return True

        command = tokens[0].lower()
        try:
            if command == "o":
                if len(tokens) != 1:
                    raise ValueError("usage: o")
                self._move_increment(self.normal_step_mm)
            elif command == "p":
                if len(tokens) != 1:
                    raise ValueError("usage: p")
                self._move_increment(self.coarse_step_mm)
            elif command == "i":
                if len(tokens) != 1:
                    raise ValueError("usage: i")
                self._move_increment(self.fine_step_mm)
            elif command == "c":
                if len(tokens) != 1:
                    raise ValueError("usage: c")
                self._move_increment(-self.normal_step_mm)
            elif command == "v":
                if len(tokens) != 1:
                    raise ValueError("usage: v")
                self._move_increment(-self.coarse_step_mm)
            elif command == "x":
                if len(tokens) != 1:
                    raise ValueError("usage: x")
                self._move_increment(-self.fine_step_mm)
            elif command in ("g", "goto"):
                if len(tokens) != 2:
                    raise ValueError("usage: g <width_mm>")
                width = float(tokens[1])
                if not math.isfinite(width):
                    raise ValueError("width must be finite")
                self._publish_target(width)
            elif command == "open":
                if len(tokens) != 1:
                    raise ValueError("usage: open")
                self._publish_target(self.max_width_mm)
            elif command == "speed":
                speed = self._parse_positive_arg(tokens)
                if not self.min_speed_mm_s <= speed <= self.max_speed_mm_s:
                    raise ValueError(
                        "speed must be in [%.3f, %.3f] mm/s" %
                        (self.min_speed_mm_s, self.max_speed_mm_s))
                self.speed_mm_s = speed
                print("speed={:.3f} mm/s".format(self.speed_mm_s))
            elif command in ("stop", "s"):
                if len(tokens) != 1:
                    raise ValueError("usage: stop")
                self._hold_current_position("operator command")
            elif command == "home":
                if len(tokens) != 1:
                    raise ValueError("usage: home")
                rospy.logwarn(
                    "auto_update has no homing service; moving to the "
                    "configured maximum width only")
                self._publish_target(self.max_width_mm)
            elif command == "ack":
                if len(tokens) != 1:
                    raise ValueError("usage: ack")
                rospy.logerr(
                    "ack is unavailable in auto_update mode; restart the "
                    "driver or use script mode for fault acknowledgement")
            elif command in ("status", "st"):
                if len(tokens) != 1:
                    raise ValueError("usage: status")
                self._print_status()
            elif command in ("help", "?"):
                self._print_help()
            elif command in ("q", "quit", "exit"):
                held = self._hold_current_position("operator exit")
                if held and self.exit_hold_wait_s > 0.0:
                    rospy.sleep(self.exit_hold_wait_s)
                self._shutdown_hold_sent = True
                return False
            else:
                print("Unknown command: {}. Enter 'help'.".format(command))
        except ValueError as exc:
            print("Invalid command: {}".format(exc))
        return True

    def _prompt_command(self, prompt, command_prefix=None):
        try:
            value = input("\n{}".format(prompt)).strip()
        except EOFError:
            return self._handle_command("q")
        except KeyboardInterrupt:
            print("\nInput cancelled")
            return True

        if not value:
            print("Input cancelled")
            return True
        if command_prefix is not None:
            value = "{} {}".format(command_prefix, value)
        return self._handle_command(value)

    def _run_key_mode(self):
        self._print_help()
        while not rospy.is_shutdown():
            try:
                key = self._read_single_key()
            except KeyboardInterrupt:
                print("")
                self._handle_command("q")
                return 0

            if not key:
                self._handle_command("q")
                return 0

            key = key.lower()
            if key in ("o", "p", "i", "c", "v", "x", "s", "q"):
                if not self._handle_command(key):
                    return 0
            elif key == "t":
                self._handle_command("status")
            elif key in ("h", "?"):
                self._print_help()
            elif key == "g":
                if not self._prompt_command(
                        "absolute width (mm)> ", command_prefix="g"):
                    return 0
            elif key == "r":
                if not self._prompt_command(
                        "movement speed (mm/s)> ", command_prefix="speed"):
                    return 0
            elif key == ":":
                if not self._prompt_command("command> "):
                    return 0
            elif key not in ("\r", "\n", " ", "\t", "\x1b"):
                print("Unknown key: {!r}. Press 'h' for help.".format(key))
        return 0

    def _run_line_mode(self):
        self._print_help()
        while not rospy.is_shutdown():
            try:
                line = input("wsg> ")
            except EOFError:
                line = "q"
            except KeyboardInterrupt:
                print("")
                line = "q"
            if not self._handle_command(line):
                return 0
        return 0

    def _on_shutdown(self):
        if self._shutdown_hold_sent:
            return
        self._shutdown_hold_sent = True
        try:
            self._hold_current_position("node shutdown")
        except Exception:
            pass

    def run(self):
        if not self._wait_until_ready():
            return 1

        rospy.loginfo(
            "Manual control ready for auto_update. width=[%.1f, %.1f] mm "
            "speed=%.1f mm/s increments=[%.1f, %.1f, %.1f] mm",
            self.min_width_mm,
            self.max_width_mm,
            self.speed_mm_s,
            self.fine_step_mm,
            self.normal_step_mm,
            self.coarse_step_mm,
        )
        self._print_status()
        if sys.stdin.isatty():
            return self._run_key_mode()
        return self._run_line_mode()


if __name__ == "__main__":
    try:
        controller = WSG50ManualControl()
        raise SystemExit(controller.run())
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as exc:
        rospy.logfatal("Manual control failed: %s", exc)
        raise
