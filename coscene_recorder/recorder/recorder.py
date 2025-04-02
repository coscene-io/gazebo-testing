#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import yaml
import signal
import sys
import atexit
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
import subprocess
import random
import string
import threading
import time
import os


@dataclass
class RecorderConfig:
    record_dir: Path
    max_split_size: int
    max_bag_duration: int
    max_cache_size: int
    topics: List[str]
    use_sim_time: bool

    @classmethod
    def from_dict(cls, config_dict: dict) -> "RecorderConfig":
        record_dir = Path(config_dict.get("record_dir", "~/rosbag2")).expanduser()
        max_split_size = int(config_dict.get("max_split_size", 0))
        max_bag_duration = int(config_dict.get("max_bag_duration", 0))
        max_cache_size = int(config_dict.get("max_cache_size", 0))
        topics = config_dict.get("topics", [])
        use_sim_time = bool(config_dict.get("use_sim_time", False))

        return cls(
            record_dir=record_dir,
            max_split_size=max_split_size,
            max_bag_duration=max_bag_duration,
            max_cache_size=max_cache_size,
            topics=topics,
            use_sim_time=use_sim_time,
        )


class RecorderNode(Node):
    def __init__(self):
        super().__init__("recorder_node")

        self._recording_stopped = False
        self._recording_process = None
        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value

        if not os.path.exists(config_file):
            raise FileNotFoundError(f"Config file not found: {config_file}")

        try:
            self.config = self.load_config(config_file)
            self.get_logger().info(f"Loaded config from {config_file}")

            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)
            atexit.register(self.ensure_stop_recording)

            self.start_recording_async()

            self.flag_monitor_thread = threading.Thread(target=self.watch_nav_done_flag, daemon=True)
            self.flag_monitor_thread.start()

            self.check_timer = self.create_timer(
                1.0,
                self.check_ros_status,
                callback_group=self.callback_group,
            )

        except Exception as e:
            self.get_logger().error(f"Failed to initialize recorder: {e}")
            raise

    def check_ros_status(self):
        if not rclpy.ok():
            self.get_logger().info("ROS context is shutting down, stopping recording")
            self.ensure_stop_recording()

    def load_config(self, config_path: str):
        try:
            with open(config_path, "r") as f:
                config_dict = yaml.safe_load(f)
                return RecorderConfig.from_dict(config_dict)
        except Exception as e:
            self.get_logger().error(f"Unable to load config file: {e}")
            raise

    def _get_unique_base_dir(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        random_suffix = "".join(random.choices(string.ascii_lowercase + string.digits, k=5))
        return self.config.record_dir / f"{timestamp}_{random_suffix}"

    def start_recording_async(self):
        def _record():
            try:
                self.output_dir = self._get_unique_base_dir()
                
                cmd = [
                    "ros2", "bag", "record",
                    "--output", str(self.output_dir),
                    "--storage", "mcap",
                    "--max-bag-size", str(self.config.max_split_size),
                    "--max-cache-size", str(self.config.max_cache_size),
                    "--max-bag-duration", str(self.config.max_bag_duration),
                ]

                if self.config.use_sim_time:
                    cmd.append("--include-hidden-topics")

                if self.config.topics:
                    cmd.extend(self.config.topics)
                else:
                    cmd.append("--all")

                with open("/tmp/latest_bag_path.txt", "w") as f:
                    f.write(str(self.output_dir))
                self.get_logger().info(f"Recording to directory: {self.output_dir}")
                self.get_logger().info(f"Running command: {' '.join(cmd)}")

                self._recording_process = subprocess.Popen(cmd)
                self._recording_process.wait()
                self.get_logger().info("Recording subprocess ended.")
            except Exception as e:
                self.get_logger().error(f"Recording thread failed: {e}")

        self.record_thread = threading.Thread(target=_record, daemon=True)
        self.record_thread.start()

    def ensure_stop_recording(self):
        if not self._recording_stopped:
            self.stop_recording()
            self._recording_stopped = True

    def stop_recording(self):
        self.get_logger().info("Stopping recording...")
        try:
            if self._recording_process and self._recording_process.poll() is None:
                self._recording_process.terminate()
                self._recording_process.wait()
                self.get_logger().info("Recording process terminated.")
            else:
                self.get_logger().info("Recording process already exited.")
            self.reindex_bag_file()
        except Exception as e:
            self.get_logger().error(f"Error stopping recording: {e}")

    def signal_handler(self, sig, frame):
        self.get_logger().info(f"Received signal {sig}, gracefully shutting down...")
        self.ensure_stop_recording()
        rclpy.shutdown()

    def reindex_bag_file(self):
        if hasattr(self, "output_dir") and self.output_dir.exists():
            try:
                self.get_logger().info(f"Attempting to reindex bag file in: {self.output_dir}")
                subprocess.run(
                    ['ros2', 'bag', 'reindex', str(self.output_dir)],
                    check=True
                )
                self.get_logger().info("Bag file reindexed successfully.")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to reindex bag file: {e}")

    def watch_nav_done_flag(self):
        flag_path = "/tmp/nav_done.flag"
        self.get_logger().info("Starting to monitor nav_done flag...")

        while not self._recording_stopped:
            if os.path.exists(flag_path):
                self.get_logger().info("Detected nav_done.flag, stopping recording...")
                self.ensure_stop_recording()
                try:
                    os.remove(flag_path)
                    self.get_logger().info("nav_done.flag removed after processing.")
                except Exception as e:
                    self.get_logger().warn(f"Failed to remove nav_done.flag: {e}")
                break
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RecorderNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.ensure_stop_recording()
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
