from datetime import datetime
from typing import Dict, Set

import rclpy
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
from node_msgs.msg import NodeList, NodeStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

# Lifecycle states mapping
LIFECYCLE_STATES = {
    0: "UNKNOWN",
    1: "UNCONFIGURED",
    2: "INACTIVE",
    3: "ACTIVE",
    4: "FINALIZED",
}


def to_relative_time(current_time, past_time):
    """Format time difference as human-readable string"""
    diff_sec = (current_time.nanoseconds - past_time.nanoseconds) / 1e9

    if diff_sec < 60:
        return f"{int(diff_sec)} seconds ago"
    elif diff_sec < 3600:
        minutes = int(diff_sec / 60)
        return f"{minutes} minute{'s' if minutes > 1 else ''} ago"
    elif diff_sec < 86400:
        hours = int(diff_sec / 3600)
        return f"{hours} hour{'s' if hours > 1 else ''} ago"
    else:
        days = int(diff_sec / 86400)
        return f"{days} day{'s' if days > 1 else ''} ago"


def to_iso8601(ros_time):
    """Convert ROS Time to ISO8601 formatted timestamp"""
    seconds = ros_time.nanoseconds // 1_000_000_000
    dt = datetime.fromtimestamp(seconds)
    return dt.strftime("%Y-%m-%dT%H:%M:%SZ")


class NodeMonitor(Node):
    def __init__(self):
        super().__init__("node_lister")

        # Define update period in seconds
        self.update_period = 1.0

        # Create callback group for concurrent operations
        self.cb_group = MutuallyExclusiveCallbackGroup()

        # Create QoS profiles
        self.node_list_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            lifespan=Duration(seconds=self.update_period),
            history=HistoryPolicy.KEEP_LAST,
        )

        # Event subscription QoS (needs only reliability)
        self.event_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # Create publisher
        self.publisher = self.create_publisher(
            NodeList, "/node_list", self.node_list_qos
        )

        # Node tracking data structures
        self.nodes: Dict[str, NodeStatus] = (
            {}
        )  # Using NodeStatus directly instead of NodeInfo
        self.lifecycle_nodes: Set[str] = set()

        # Subscribe to lifecycle transition events
        self.transition_event_sub = self.create_subscription(
            TransitionEvent,
            "/lifecycle_events",
            self.on_transition_event,
            self.event_qos,
            callback_group=self.cb_group,
        )

        # Set up timer for periodic updates
        self.timer = self.create_timer(
            self.update_period, self.timer_callback, callback_group=self.cb_group
        )

        self.get_logger().info("Node Monitor started")

    def on_transition_event(self, msg: TransitionEvent):
        """Handle lifecycle transition events"""
        node_id = msg.id
        state_label = LIFECYCLE_STATES.get(msg.goal_state.id, "UNKNOWN")

        # Mark as lifecycle node and update state
        self.lifecycle_nodes.add(node_id)

        if node_id in self.nodes:
            node_status = self.nodes[node_id]
            node_status.node_type = "lifecycle"
            node_status.state = state_label

            # Update last_seen
            now = self.get_clock().now()
            node_status.last_seen = to_iso8601(now)
            node_status.relative_time = "just now"

    def check_is_lifecycle_node(self, node_name: str) -> bool:
        """Check if a node is a lifecycle node (only for previously unknown nodes)"""
        # If we've already identified it, don't check again
        if node_name in self.lifecycle_nodes:
            return True

        # Only check new nodes via service
        service_name = f"{node_name}/get_state"
        client = self.create_client(
            GetState, service_name, callback_group=self.cb_group
        )

        try:
            if client.wait_for_service(timeout_sec=0.1):
                self.lifecycle_nodes.add(node_name)
                return True
        finally:
            self.destroy_client(client)

        return False

    def timer_callback(self):
        """Periodic update of node list"""
        current_time = self.get_clock().now()
        current_node_names = set()

        # Get current nodes
        for name, namespace in self.get_node_names_and_namespaces():
            # Form fully qualified name
            full_name = (
                f"{namespace.rstrip('/')}/{name}"
                if namespace and namespace != "/"
                else f"/{name}"
            )

            # Skip ourselves
            if full_name == self.get_fully_qualified_name():
                continue

            current_node_names.add(full_name)

            # Check node type and update info
            is_lifecycle = self.check_is_lifecycle_node(full_name)

            # Update or create NodeStatus for this node
            if full_name in self.nodes:
                node_status = self.nodes[full_name]
                node_status.node_type = "lifecycle" if is_lifecycle else "regular"

                # Only update state if not already set by lifecycle events
                if not is_lifecycle or node_status.state == "UNKNOWN":
                    node_status.state = "ACTIVE"

                # Update time info
                node_status.last_seen = to_iso8601(current_time)
                node_status.relative_time = "just now"
            else:
                # Create new NodeStatus message
                node_status = NodeStatus()
                node_status.name = full_name
                node_status.state = "ACTIVE"
                node_status.node_type = "lifecycle" if is_lifecycle else "regular"
                node_status.last_seen = to_iso8601(current_time)
                node_status.relative_time = "just now"
                self.nodes[full_name] = node_status

        # Update nodes that disappeared
        self._update_disappeared_nodes(current_node_names, current_time)

        # Build and publish message
        self._publish_node_list()

    def _update_disappeared_nodes(self, current_node_names, current_time):
        """Update state of nodes that have disappeared"""
        for node_name, node_status in self.nodes.items():
            if node_name not in current_node_names:
                # Set appropriate state based on node type
                if node_status.node_type == "lifecycle":
                    if node_status.state in ["ACTIVE", "INACTIVE", "UNCONFIGURED"]:
                        node_status.state = "FINALIZED"
                else:
                    node_status.state = "INACTIVE"

                # Update the relative time for disappeared nodes (need to parse last_seen)
                # For simplicity, let's just update relative time directly
                try:
                    # Parse last_seen string to get a timestamp reference
                    node_status.relative_time = to_relative_time(
                        current_time, self._timestamp_to_ros_time(node_status.last_seen)
                    )
                except:
                    # Fallback if parsing fails
                    node_status.relative_time = "-"

    def _timestamp_to_ros_time(self, timestamp_str):
        """Convert ISO8601 timestamp to ROS Time (approximate)"""
        try:
            dt = datetime.strptime(timestamp_str, "%Y-%m-%dT%H:%M:%SZ")
            secs = int(dt.timestamp())
            return Time(seconds=secs)
        except:
            return self.get_clock().now()  # Fallback

    def _publish_node_list(self):
        """Build and publish the node list message"""
        node_list_msg = NodeList()
        node_list_msg.nodes = list(self.nodes.values())
        node_list_msg.total_nodes = len(node_list_msg.nodes)

        # Count active vs inactive nodes
        active_nodes = sum(1 for node in node_list_msg.nodes if node.state == "ACTIVE")
        node_list_msg.active_nodes = active_nodes
        node_list_msg.inactive_nodes = node_list_msg.total_nodes - active_nodes

        # Publish the message
        self.publisher.publish(node_list_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NodeMonitor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()
