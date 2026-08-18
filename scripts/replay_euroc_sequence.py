#!/usr/bin/env python3
"""Replay a planarized EuRoC proxy stream into the existing Aegis ROS topics."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class EurocReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("euroc_replay_node")
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.truth_pub = self.create_publisher(PoseStamped, "/ground_truth/pose", 10)
        self.published_samples = 0
        self.first_timestamp = None
        self.last_timestamp = None
        self.subscriber_snapshot = {
            "odom": 0,
            "imu": 0,
            "truth": 0,
        }

    def wait_for_subscribers(self, timeout_sec: float = 10.0) -> None:
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout_sec:
            odom_ready = self.odom_pub.get_subscription_count() > 0
            imu_ready = self.imu_pub.get_subscription_count() > 0
            truth_ready = self.truth_pub.get_subscription_count() > 0
            self.subscriber_snapshot = {
                "odom": self.odom_pub.get_subscription_count(),
                "imu": self.imu_pub.get_subscription_count(),
                "truth": self.truth_pub.get_subscription_count(),
            }
            if odom_ready and imu_ready and truth_ready:
                self.get_logger().info("Replay subscribers connected; starting EuRoC stream")
                return
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().warning(
            "Timed out waiting for all replay subscribers; continuing anyway "
            f"(odom={self.subscriber_snapshot['odom']}, "
            f"imu={self.subscriber_snapshot['imu']}, "
            f"truth={self.subscriber_snapshot['truth']})"
        )

    def replay(self, proxy_csv_path: Path, sleep_scale: float, max_samples: int | None = None) -> dict[str, object]:
        self.wait_for_subscribers()
        start_monotonic = time.monotonic()
        with proxy_csv_path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            previous_timestamp = None
            for sample_index, row in enumerate(reader):
                if max_samples is not None and sample_index >= max_samples:
                    break
                timestamp = float(row["timestamp"])
                x = float(row["x"])
                y = float(row["y"])
                yaw = float(row["yaw"])
                vx = float(row["vx"])
                vy = float(row["vy"])
                imu_omega_z = float(row["imu_omega_z"])

                if previous_timestamp is not None:
                    dt = max(timestamp - previous_timestamp, 0.0)
                    if sleep_scale > 0.0:
                        time.sleep(dt / sleep_scale)
                previous_timestamp = timestamp

                stamp = rclpy.time.Time(seconds=timestamp).to_msg()
                qx, qy, qz, qw = yaw_to_quaternion(yaw)

                truth = PoseStamped()
                truth.header.stamp = stamp
                truth.header.frame_id = "map"
                truth.pose.position.x = x
                truth.pose.position.y = y
                truth.pose.orientation.x = qx
                truth.pose.orientation.y = qy
                truth.pose.orientation.z = qz
                truth.pose.orientation.w = qw
                self.truth_pub.publish(truth)

                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy
                # Current Aegis ROS wrappers read angular rate from /odom twist.
                odom.twist.twist.angular.z = imu_omega_z
                self.odom_pub.publish(odom)

                imu = Imu()
                imu.header.stamp = stamp
                imu.header.frame_id = "imu_link"
                imu.angular_velocity.z = imu_omega_z
                self.imu_pub.publish(imu)

                if self.first_timestamp is None:
                    self.first_timestamp = timestamp
                self.last_timestamp = timestamp
                self.published_samples += 1
                rclpy.spin_once(self, timeout_sec=0.0)

        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        return {
            "published_samples": self.published_samples,
            "first_timestamp": self.first_timestamp,
            "last_timestamp": self.last_timestamp,
            "subscriber_snapshot": self.subscriber_snapshot,
            "elapsed_wall_seconds": time.monotonic() - start_monotonic,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--proxy-csv", required=True)
    parser.add_argument("--sleep-scale", type=float, default=30.0, help="Replay speedup factor. Higher is faster.")
    parser.add_argument("--max-samples", type=int, default=None, help="Optional limit for replayed samples.")
    parser.add_argument("--summary-out", default=None, help="Optional JSON summary path.")
    args = parser.parse_args()

    rclpy.init()
    node = EurocReplayNode()
    try:
        summary = node.replay(Path(args.proxy_csv), args.sleep_scale, args.max_samples)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.summary_out:
        summary_path = Path(args.summary_out)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        with summary_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
