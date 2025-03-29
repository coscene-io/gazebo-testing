#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosbag2_py import RecordOptions, StorageOptions, Recorder, Reindexer
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
import os


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

        # Flag to track if recording has been stopped
        self._recording_stopped = False

        # Callback group for periodic checks
        self.callback_group = MutuallyExclusiveCallbackGroup()

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

            # Register cleanup function for Python exit
            atexit.register(self.ensure_stop_recording)

            # Start recording
            self.start_recording()

            # Add a timer to check ROS status
            self.check_timer = self.create_timer(
                1.0,  # Check every second
                self.check_ros_status,
                callback_group=self.callback_group,
            )

        except Exception as e:
            self.get_logger().error(f"Failed to initialize recorder: {e}")
            raise

    def check_ros_status(self):
        """Check ROS status and stop recording if ROS is shutting down"""
        if not rclpy.ok():
            self.get_logger().info("ROS context is shutting down, stopping recording")
            self.ensure_stop_recording()

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
            self.output_dir = self._get_unique_base_dir()  
            storage_options = StorageOptions(
                uri=str(self.output_dir),
                storage_id="mcap",
                max_bagfile_size=self.config.max_split_size,
                max_bagfile_duration=self.config.max_bag_duration,
            )
            self.get_logger().info(f"Recording to directory: {storage_options.uri}")

            # write the mcap folder path to a file. This temporary file is used by the reindex script
            try:
                with open("/tmp/latest_bag_path.txt", "w") as f:
                    f.write(str(self.output_dir))
                self.get_logger().info(f"Saved latest bag path to /tmp/latest_bag_path.txt")
            except Exception as e:
                self.get_logger().warn(f"Failed to write latest bag path file: {e}")
            

            self.recorder.record(
                storage_options,
                self.config.record_options,
            )
            self.get_logger().info(f"Recording started with config: {self.config}")
        except Exception as e:
            self.get_logger().error(f"Critical recording failure: {e}")
            sys.exit(1)

    def ensure_stop_recording(self):
        """Ensure stop_recording is only called once"""
        if not self._recording_stopped:
            self.stop_recording()
            self._recording_stopped = True

    def stop_recording(self):
        self.get_logger().info("Stopping recording...")
        try:
            self.recorder.stop()
            self.get_logger().info("Recording stopped successfully")
        except Exception as e:
            self.get_logger().error(f"Error stopping recording: {e}")
        finally:
            self.reindex_bag_file()

    def signal_handler(self, sig, frame):
        """Handle signals for graceful shutdown"""
        self.get_logger().info(f"Received signal {sig}, gracefully shutting down...")
        # First stop recording
        self.ensure_stop_recording()
        # Then shutdown ROS
        rclpy.shutdown()
    
    def reindex_bag_file(self):
        if hasattr(self, "output_dir") and self.output_dir.exists():
            try:
                self.get_logger().info(f"Attempting to reindex bag file in: {self.output_dir}")
                
                storage_options = StorageOptions(
                    uri=str(self.output_dir),
                    storage_id="mcap"
                )
                
                reindexer = Reindexer()
                reindexer.reindex(storage_options)
                
                self.get_logger().info("Bag file reindexed successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to reindex bag file: {e}")


def main(args=None):
    rclpy.init(args=args)  # This handles ROS2-specific command line arguments

    try:
        node = RecorderNode()

        # Use single-threaded executor
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            # Ensure cleanup before shutdown
            node.ensure_stop_recording()
            executor.shutdown()
            node.destroy_node()

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        # Final cleanup
        rclpy.shutdown()


if __name__ == "__main__":
    main()
