#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32

def main():
    rospy.init_node("force_ramp_pub")
    topic   = rospy.get_param("~topic", "/znsv6_control")
    rate_hz = float(rospy.get_param("~rate_hz", 30.0))
    slope   = float(rospy.get_param("~slope_N_per_s", 0.5))   # 斜坡速度 N/s
    peak    = float(rospy.get_param("~peak_N", 5.0))          # 峰值 N
    loop    = bool(rospy.get_param("~loop", False))           # 是否循环 0→peak→0→...
    hold_s  = float(rospy.get_param("~hold_s", 5.0))          # [MOD] 峰值保持时长（秒），默认 5s

    pub = rospy.Publisher(topic, Float32, queue_size=10)
    r = rospy.Rate(rate_hz)

    dt  = 1.0 / rate_hz
    val = 0.0
    dirn = +1.0   # +1 上升，-1 下降
    state = "up"  # [MOD] 三状态：'up' → 'hold' → 'down'
    hold_t = 0.0  # [MOD] 已保持时间累计

    while not rospy.is_shutdown():
        pub.publish(Float32(data=val))

        if state == "up":
            val += slope * dt
            if val >= peak:
                val = peak
                state = "hold"   # [MOD] 进入保持阶段
                hold_t = 0.0

        elif state == "hold":
            hold_t += dt
            # 保持在峰值不变
            val = peak
            if hold_t >= hold_s:
                state = "down"   # [MOD] 保持完毕开始下降

        elif state == "down":
            val -= slope * dt
            if val <= 0.0:
                val = 0.0
                if loop:
                    state = "up"   # 循环：重新上升
                else:
                    # 结束前再发几帧 0，保证对端收齐
                    for _ in range(3):
                        pub.publish(Float32(data=0.0)); r.sleep()
                    break

        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

