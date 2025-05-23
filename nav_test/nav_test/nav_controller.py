#!/usr/bin/env python3
import json
import os
import random
import time
import subprocess
import signal
from datetime import datetime
from uuid import uuid4

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from nav_test.ros_utils import load_pose


class TaskManager:
    def __init__(self, config_path):
        self.config_path = config_path
        self.all_points = []
        self.task_queue = []
        self.history = []
        self.load_config()

    def load_config(self):
        try:
            with open(self.config_path, "r") as f:
                data = yaml.safe_load(f)
                self.all_points = data["points"]
        except Exception as e:
            raise RuntimeError(f"Configuration file loading failed: {str(e)}")

    def generate_task(self, num=10):
        if len(self.all_points) < num:
            raise ValueError(
                f"Configuration file contains only {len(self.all_points)} points, at least {num} points are required"
            )

        selected = random.sample(self.all_points, num)
        self.task_queue = [
            {
                "id": f"TASK-{uuid4().hex[:6]}",
                "point": p,
                "status": "pending",
                "start_time": None,
                "end_time": None,
                "error_info": "",
            }
            for p in selected
        ]

    def get_next_task(self):
        return self.task_queue[0] if self.task_queue else None

    def mark_task_complete(self, task_id, success=True, error_info=""):
        for idx, task in enumerate(self.task_queue):
            if task["id"] == task_id:
                task["status"] = "success" if success else "failed"
                task["end_time"] = datetime.now()
                task["error_info"] = error_info
                completed = self.task_queue.pop(idx)
                self.history.append(completed)
                return completed
        return None


class NavController(Node):
    def __init__(self):
        super().__init__("nav_controller")

        self.declare_parameter("config_path", "")
        self.declare_parameter("report_path", "/home/qingyu")

        try:
            # Get configuration file path
            self.config_path = self.get_parameter("config_path").value
            print("The config path is: " + self.config_path)
            if not os.path.exists(self.config_path):
                raise FileNotFoundError(
                    f"Configuration file not found: {self.config_path}"
                )

            # Initialize task manager
            self.task_mgr = TaskManager(self.config_path)
            self.task_mgr.generate_task()

        except Exception as e:
            self.get_logger().fatal(f"Initialization failed: {str(e)}")
            raise

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.task_success = 0
        self.task_fail = 0

        qos = QoSProfile(depth=10)
        self.sub_error = self.create_subscription(
            String, "/error_status", self.error_callback, qos
        )
        self.pub_error = self.create_publisher(String, "/error_status", qos)

        self.current_goal_handle = None
        self.stop_navigation = False
        self.current_task = None

        # Track if amcl_pose has been received after publishing initial pose
        self.amcl_pose_received = False
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            QoSProfile(depth=10)
        )

        # Schedule startup sequence with a short delay to allow node initialization to complete
        self.startup_timer = self.create_timer(0.5, self.initialize_sequence)

    def initialize_sequence(self):
        # This method will run after node initialization is complete
        # Ensure it only runs once by destroying the timer
        self.destroy_timer(self.startup_timer)

        # Wait for the navigation system to be ready
        self.get_logger().info("Waiting for navigation server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Navigation server ready.")

        # Check if AMCL is ready, publish initial pose, then start tasks
        self.publish_initial_pose_and_start()

    def publish_initial_pose_and_start(self):
        """Publish the initial pose and then start navigation tasks"""
        try:
            # Load configuration file
            with open(self.config_path, "r") as f:
                config = yaml.safe_load(f)

            # Use load_pose function to process configuration
            initial_pose_msg = load_pose(config.get("initial_pose"))

            # Set current timestamp to ensure message freshness
            initial_pose_msg.header.stamp = self.get_clock().now().to_msg()

            # Create publisher using AMCL's expected QoS settings
            initial_pose_qos = QoSProfile(depth=10)
            initial_pose_qos.reliability = ReliabilityPolicy.RELIABLE
            initial_pose_qos.durability = DurabilityPolicy.VOLATILE

            pub = self.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", initial_pose_qos
            )

            # First, check if the topic has subscribers (AMCL is listening)
            self.get_logger().info(
                "Checking if AMCL is ready to receive initial pose..."
            )
            timeout = 10.0  # seconds
            start_time = time.time()
            while pub.get_subscription_count() == 0:
                if time.time() - start_time > timeout:
                    self.get_logger().warn(
                        "No subscribers found for /initialpose after waiting. Publishing anyway."
                    )
                    break
                time.sleep(0.1)

            # Log subscriber count for debugging
            sub_count = pub.get_subscription_count()
            self.get_logger().info(
                f"Found {sub_count} subscribers for /initialpose topic"
            )

            # Reset amcl_pose_received flag before publishing
            self.amcl_pose_received = False

            # Loop to publish initial pose until AMCL acknowledges
            max_attempts = 20
            interval = 0.5
            attempts = 0
            while not self.amcl_pose_received and attempts < max_attempts:
                pub.publish(initial_pose_msg)
                self.get_logger().info(f"Published initial pose, attempt {attempts + 1}")
                time.sleep(interval)
                attempts += 1

            if not self.amcl_pose_received:
                self.get_logger().warn("AMCL did not acknowledge initial pose within timeout.")
            else:
                self.get_logger().info("AMCL acknowledged initial pose.")


            # Clean up publisher
            self.destroy_publisher(pub)

            # Start navigation tasks
            self.get_logger().info("Starting navigation tasks...")
            self.start_next_task()

        except Exception as e:
            self.get_logger().error(f"Failed to publish initial pose: {str(e)}")
            raise
    
    def amcl_pose_callback(self, msg):
        self.amcl_pose_received = True


    def start_next_task(self):
        if self.stop_navigation:
            return

        next_task = self.task_mgr.get_next_task()
        if not next_task:
            self.generate_report()
            self.destroy_node()
            return

        self.current_task = next_task
        self.current_task["start_time"] = datetime.now()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.current_task["point"]["x"]
        goal_msg.pose.pose.position.y = self.current_task["point"]["y"]
        goal_msg.pose.pose.orientation.w = self.current_task["point"].get("w", 1.0)

        self.get_logger().info(f"Starting task {self.current_task['id']}")
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.handle_task_failure("Goal rejected by server")
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result_wrapper = future.result()

            # Get action final status
            goal_status = result_wrapper.status

            # Status code mapping
            status_map = {
                GoalStatus.STATUS_UNKNOWN: "Unknown status",
                GoalStatus.STATUS_ACCEPTED: "Accepted",
                GoalStatus.STATUS_EXECUTING: "Executing",
                GoalStatus.STATUS_CANCELING: "Canceling",
                GoalStatus.STATUS_SUCCEEDED: "Navigation succeeded",
                GoalStatus.STATUS_CANCELED: "Canceled",
                GoalStatus.STATUS_ABORTED: "Target point unreachable, abandoned",
            }

            self.get_logger().info(
                f"Navigation final status: {status_map.get(goal_status, 'Unknown status')} ({goal_status})"
            )

            if goal_status == GoalStatus.STATUS_SUCCEEDED:
                self.handle_task_success()
            else:
                self.handle_task_failure(status_map[goal_status])

        except Exception as e:
            error_msg = f"Result processing exception: {str(e)}"
            self.get_logger().error(error_msg)
            self.handle_task_failure(error_msg)

    def handle_task_success(self):
        self.task_mgr.mark_task_complete(self.current_task["id"])
        self.task_success += 1
        self.get_logger().info(f"Task {self.current_task['id']} completed successfully")
        self.start_next_task()

    def handle_task_failure(self, reason):
        error_msg = String()
        error_msg.data = f"code:10001,info:{reason},value:1"
        self.pub_error.publish(error_msg)

        self.task_mgr.mark_task_complete(
            self.current_task["id"], success=False, error_info=reason
        )
        self.task_fail += 1
        self.get_logger().error(f"Task failed: {reason}")
        self.start_next_task()

    def error_callback(self, msg):
        error_dict = {}
        try:
            for item in msg.data.split(","):
                if ":" not in item:
                    continue
                key, value = item.split(":", 1)
                error_dict[key.strip()] = value.strip()

            code = int(error_dict.get("code", 0))
            value = error_dict.get("value", "0")

            if 1000 <= code <= 1999:
                if value == "1":
                    self.emergency_stop()
                elif value == "0" and self.stop_navigation:
                    self.resume_navigation()

        except Exception as e:
            self.get_logger().error(
                f"Error parsing failed: {str(e)} Original data: {msg.data}"
            )

    def emergency_stop(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.stop_navigation = True
        self.get_logger().warn("! Emergency stop activated !")

    def resume_navigation(self):
        self.stop_navigation = False
        self.get_logger().info("System resumed normal operation")
        if not self.current_goal_handle:
            self.start_next_task()

    def generate_report(self):
        try:
            report_dir = self.get_parameter("report_path").value
            report_filename = f"nav_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xml"
            report_path = os.path.join(report_dir, report_filename)

            os.makedirs(report_dir, exist_ok=True)

            import xml.etree.ElementTree as ET
            from xml.dom import minidom

            for task in self.task_mgr.history:
                if not isinstance(task.get("start_time"), datetime):
                    self.get_logger().error(f"Invalid start_time in task {task['id']}")
                    task["start_time"] = datetime.now()
                if not isinstance(task.get("end_time"), datetime):
                    self.get_logger().error(f"Invalid end_time in task {task['id']}")
                    task["end_time"] = datetime.now()

            total_tasks = len(self.task_mgr.history)
            failures = sum(1 for t in self.task_mgr.history if t["status"] == "failed")
            total_time = sum(
                (task["end_time"] - task["start_time"]).total_seconds()
                for task in self.task_mgr.history
            )

            testsuites = ET.Element("testsuites", {
                "id": f"NAV-REPORT-{datetime.now().strftime('%Y%m%d%H%M%S')}",
                "name": "Navigation Test Suite",
                "tests": str(total_tasks),
                "failures": str(failures),
                "time": f"{total_time:.3f}"
            })

            testsuite = ET.SubElement(testsuites, "testsuite", {
                "id": "navigation_tasks",
                "name": "Navigation Tasks",
                "tests": str(total_tasks),
                "failures": str(failures),
                "time": f"{total_time:.3f}"
            })

            for task in self.task_mgr.history:
                duration = (task["end_time"] - task["start_time"]).total_seconds()
                case_attrs = {
                    "id": task["id"],
                    "name": f"Navigation to ({task['point']['x']}, {task['point']['y']})",
                    "time": f"{duration:.3f}"
                }
                testcase = ET.SubElement(testsuite, "testcase", case_attrs)

                if task["status"] == "failed":
                    failure = ET.SubElement(testcase, "failure", {
                        "message": task["error_info"],
                        "type": "NavigationError"
                    })
                    failure.text = f"{task['error_info']}\n" \
                                f"Target Coordinates: ({task['point']['x']}, {task['point']['y']})\n" \
                                f"Duration: {duration:.2f}s"

            xml_str = ET.tostring(testsuites, encoding="utf-8")
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="  ", encoding="utf-8")

            with open(report_path, "wb") as f:
                f.write(pretty_xml)

            self.get_logger().info(f"JUnit report generated: {report_path}")
            rclpy.try_shutdown()

            # --- Add code to kill parent process (ros2 launch) ---
            try:
                ppid = os.getppid()
                self.get_logger().info(f"Killing parent process {ppid} to shutdown launch system...")
                os.kill(ppid, signal.SIGINT)
            except Exception as e:
                self.get_logger().error(f"Failed to kill parent process: {e}")

        except Exception as e:
            self.get_logger().error(f"Report generation failed: {str(e)}")
            raise  




def main(args=None):
    try:
        rclpy.init(args=args)
        node = NavController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Ensure the node is destroyed before calling shutdown
        if "node" in locals():
            node.destroy_node()
        # Check if ROS context is initialized before shutdown
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
