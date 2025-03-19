#!/usr/bin/env python3

import argparse
import yaml
import signal
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
import rclpy
from rclpy.node import Node
from rosbag2_py import RecordOptions, StorageOptions, Recorder


@dataclass
class RecorderConfig:
    """Configuration for ROS2 bag recorder"""

    record_dir: Path
    topics: List[str]
    max_split_size: int
    storage_format: str
    compression_format: str
    compression_mode: str
    max_bag_duration: int
    storage_options: Optional[StorageOptions] = None
    record_options: Optional[RecordOptions] = None

    @classmethod
    def from_dict(cls, config_dict: dict) -> "RecorderConfig":
        """Create a RecorderConfig from a dictionary"""
        record_dir = Path(config_dict.get("record_dir", "~/rosbag2")).expanduser()

        topics = config_dict.get("topics", [])
        max_split_size = int(config_dict.get("max_split_size", "10071520"))
        storage_format = config_dict.get("storage_format", "mcap")
        compression_format = config_dict.get("compression_format", "zstd")
        compression_mode = config_dict.get("compression_mode", "file")
        max_bag_duration = int(config_dict.get("max_bag_duration", "0"))

        # Create record options
        record_options = RecordOptions()
        record_options.all = len(topics) == 0
        record_options.topics = topics

        # Set compression options based on storage format
        if storage_format.lower() == "mcap":
            record_options.compression_mode = "file"  # Use file/chunk compression
            record_options.compression_format = compression_format or "zstd"
        else:
            # For other formats, use the configured compression settings
            record_options.compression_mode = compression_mode
            record_options.compression_format = compression_format

        # Create storage options
        storage_options = StorageOptions(
            str(record_dir),  # uri
            storage_format,  # storage_id
            max_split_size,  # max_bagfile_size
            max_bag_duration,  # max_bagfile_duration
        )

        return cls(
            record_dir=record_dir,
            topics=topics,
            max_split_size=max_split_size,
            storage_format=storage_format,
            compression_format=compression_format,
            compression_mode=compression_mode,
            max_bag_duration=max_bag_duration,
            storage_options=storage_options,
            record_options=record_options,
        )


def load_config(config_path: str) -> RecorderConfig:
    """Load configuration from YAML file"""
    try:
        with open(config_path, "r") as f:
            config_dict = yaml.safe_load(f)
            return RecorderConfig.from_dict(config_dict)
    except Exception as e:
        print(f"Unable to load config file: {e}", file=sys.stderr)
        raise


def parse_args() -> argparse.Namespace:
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="ROS2 Bag Recorder")
    parser.add_argument(
        "-c",
        "--config",
        default="recorder_config.yaml",
        help="Config file path (default: recorder_config.yaml)",
    )
    return parser.parse_args()


class BagRecorderNode(Node):
    def __init__(self, config: RecorderConfig):
        super().__init__("bag_recorder_node")

        # Store configuration
        self.config = config

        # Create recording directory
        self.config.record_dir.mkdir(parents=True, exist_ok=True)

        # Initialize recorder
        self.recorder = Recorder()

        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def start_recording(self) -> None:
        try:
            self.recorder.record(
                self.config.storage_options,
                self.config.record_options,
            )
            self.get_logger().info(f"Recording configuration:\n{self.config}")
        except Exception as e:
            self.get_logger().error(f"Critical recording failure: {e}")
            sys.exit(1)  # Non-zero exit code signals error to container system

    def stop_recording(self) -> None:
        self.get_logger().info("Stopping recording...")
        try:
            self.recorder.stop()
            self.get_logger().info("Recording stopped successfully")
        except Exception as e:
            self.get_logger().error(f"Error stopping recording: {e}")

    def signal_handler(self, sig, frame) -> None:
        """Handle signals for graceful shutdown"""
        self.get_logger().info(f"Received signal {sig}, gracefully shutting down...")
        self.stop_recording()
        rclpy.shutdown()
        sys.exit(0)


def main() -> None:
    # Parse command line arguments
    args = parse_args()

    try:
        # Load configuration
        config = load_config(args.config)

        # Initialize ROS2
        rclpy.init()

        # Create recording node
        node = BagRecorderNode(config)

        # Start recording
        node.start_recording()

        # Enter ROS2 event loop
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Received KeyboardInterrupt, terminating recording")
        finally:
            # Clean up
            node.stop_recording()
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        print(f"Error executing program: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
