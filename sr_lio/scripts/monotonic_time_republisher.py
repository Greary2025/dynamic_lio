#!/usr/bin/env python3

import copy
import math
import threading

import rospy
from sensor_msgs.msg import Imu, PointCloud2


class MonotonicStampMapper:
    def __init__(self):
        self._lock = threading.Lock()
        self._input_zero = None
        self._output_zero = None
        self._last_output = None
        self._min_step = rospy.Duration.from_sec(rospy.get_param("~min_step_sec", 1e-6))
        self._initialized = False

    def remap(self, stamp):
        now = rospy.Time.now()
        stamp_sec = stamp.to_sec()
        if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            stamp_sec = now.to_sec()

        with self._lock:
            if self._input_zero is None:
                self._input_zero = stamp_sec
                self._output_zero = now
                candidate = self._output_zero
            else:
                candidate = self._output_zero + rospy.Duration.from_sec(stamp_sec - self._input_zero)

            if self._last_output is not None and candidate <= self._last_output:
                candidate = self._last_output + self._min_step

            self._last_output = candidate

            if not self._initialized:
                rospy.loginfo(
                    "Monotonic time republisher initialized: input_zero=%.6f, output_zero=%.6f",
                    self._input_zero,
                    self._output_zero.to_sec(),
                )
                self._initialized = True

            return candidate


class MonotonicTimeRepublisher:
    def __init__(self):
        lidar_in = rospy.get_param("~lidar_in_topic", "/xianfeng/lidar")
        imu_in = rospy.get_param("~imu_in_topic", "/xianfeng/imu")
        lidar_out = rospy.get_param("~lidar_out_topic", "/xianfeng/monotonic_lidar")
        imu_out = rospy.get_param("~imu_out_topic", "/xianfeng/monotonic_imu")

        self._mapper = MonotonicStampMapper()

        self._lidar_pub = rospy.Publisher(lidar_out, PointCloud2, queue_size=20)
        self._imu_pub = rospy.Publisher(imu_out, Imu, queue_size=500)

        self._lidar_sub = rospy.Subscriber(lidar_in, PointCloud2, self._handle_lidar, queue_size=20)
        self._imu_sub = rospy.Subscriber(imu_in, Imu, self._handle_imu, queue_size=1000)

        rospy.loginfo(
            "Monotonic time republisher started: %s -> %s, %s -> %s",
            lidar_in,
            lidar_out,
            imu_in,
            imu_out,
        )

    def _rewrite(self, msg):
        msg_out = copy.deepcopy(msg)
        msg_out.header.stamp = self._mapper.remap(msg.header.stamp)
        return msg_out

    def _handle_lidar(self, msg):
        self._lidar_pub.publish(self._rewrite(msg))

    def _handle_imu(self, msg):
        self._imu_pub.publish(self._rewrite(msg))


if __name__ == "__main__":
    rospy.init_node("monotonic_time_republisher")
    MonotonicTimeRepublisher()
    rospy.spin()
