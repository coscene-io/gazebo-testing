#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosbag2_py import RecordOptions, StorageOptions, Recorder
import yaml
import signal
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
import random
import string


@dataclass
class RecorderConfig:
    record_dir: Path
    max_split_size: int
    max_bag_duration: int
    record_options: Optional[RecordOptions] = None

    @classmethod
    def from_dict(cls, config_dict: dict) -> "RecorderConfig":
        """Create a RecorderConfig from a dictionary"""
        record_dir = Path(config_dict.get("record_dir", "~/rosbag2")).expanduser()

        max_split_size = int(config_dict.get("max_split_size", "10071520"))
        max_bag_duration = int(config_dict.get("max_bag_duration", "0"))
        topics = config_dict.get("topics", [])
        compression_format = config_dict.get("compression_format", None)
        use_sim_time = bool(config_dict.get("use_sim_time", False))

        # Create record options
        record_options = RecordOptions()
        record_options.all = len(topics) == 0
        record_options.topics = topics
        record_options.use_sim_time = use_sim_time

        # Set compression options based on storage format
        if compression_format is not None:
            record_options.compression_format = compression_format
            record_options.compression_mode = "file"  # Use file/chunk compression

        return cls(
            record_dir=record_dir,
            max_split_size=max_split_size,
            max_bag_duration=max_bag_duration,
            record_options=record_options,
        )


class RecorderNode(Node):
    def __init__(self):
        super().__init__("recorder_node")

        # Declare parameters
        self.declare_parameter("config_file", "config/recorder_config.yaml")
        config_file = self.get_parameter("config_file").value

        try:
            # Load configuration
            self.config = self.load_config(config_file)
            self.get_logger().info(f"Loaded config from {config_file}")

            # Initialize recorder
            self.recorder = Recorder()

            # Register signal handlers
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)

            # Start recording
            self.start_recording()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize recorder: {e}")
            raise

    def load_config(self, config_path: str):
        """Load configuration from YAML file"""
        try:
            with open(config_path, "r") as f:
                config_dict = yaml.safe_load(f)
                return RecorderConfig.from_dict(config_dict)
        except Exception as e:
            self.get_logger().error(f"Unable to load config file: {e}")
            raise

    def _get_unique_base_dir(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        random_suffix = "".join(
            random.choices(string.ascii_lowercase + string.digits, k=5)
        )
        return self.config.record_dir / f"{timestamp}_{random_suffix}"

    def start_recording(self):
        try:
            storage_options = StorageOptions(
                uri=str(self._get_unique_base_dir()),  # Convert PosixPath to string
                storage_id="mcap",
                max_bagfile_size=self.config.max_split_size,
                max_bagfile_duration=self.config.max_bag_duration,
            )
            self.get_logger().info(f"Recording to directory: {storage_options.uri}")

            # Start recording with the new storage options
            self.recorder.record(
                storage_options,
                self.config.record_options,
            )
            self.get_logger().info(f"Recording started with config: {self.config}")
        except Exception as e:
            self.get_logger().error(f"Critical recording failure: {e}")
            sys.exit(1)

    def stop_recording(self):
        self.get_logger().info("Stopping recording...")
        try:
            self.recorder.stop()
            self.get_logger().info("Recording stopped successfully")
        except Exception as e:
            self.get_logger().error(f"Error stopping recording: {e}")

    def signal_handler(self, sig, frame):
        """Handle signals for graceful shutdown"""
        self.get_logger().info(f"Received signal {sig}, gracefully shutting down...")
        self.stop_recording()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)  # This handles ROS2-specific command line arguments

    try:
        node = RecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        # Clean up
        if "node" in locals():
            node.stop_recording()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
