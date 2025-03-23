import rclpy
from node_msgs.msg import NodeList, NodeStatus
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from dataclasses import dataclass
from typing import Dict, Optional
from lifecycle_msgs.msg import State, TransitionEvent
from lifecycle_msgs.srv import GetState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
from datetime import datetime


# Lifecycle states as defined in lifecycle_msgs/State.msg
LIFECYCLE_STATES = {
    0: "UNKNOWN",
    1: "UNCONFIGURED",
    2: "INACTIVE",
    3: "ACTIVE",
    4: "FINALIZED",
}


# Define this after NodeStatus is imported
def get_node_status_states():
    return {
        "UNKNOWN": NodeStatus.STATE_UNKNOWN,
        "UNCONFIGURED": NodeStatus.STATE_UNCONFIGURED,
        "INACTIVE": NodeStatus.STATE_INACTIVE,
        "ACTIVE": NodeStatus.STATE_ACTIVE,
        "FINALIZED": NodeStatus.STATE_FINALIZED,
    }


@dataclass
class NodeInfo:
    """Dataclass to store node information with timestamps and relative time"""

    name: str  # Node name
    last_seen: Time  # Last time node was seen
    is_lifecycle_node: bool = False  # Whether this is a lifecycle node
    state: str = "UNKNOWN"  # State as a string constant
    relative_time: str = "unknown"  # Human-readable relative time

    def get_state_value(self):
        """Get the numeric state value for the message based on node type"""
        if not self.is_lifecycle_node:
            # For regular nodes: only ACTIVE or INACTIVE (if disappeared)
            if self.state == "ACTIVE":
                return NodeStatus.STATE_ACTIVE
            else:
                return NodeStatus.STATE_INACTIVE
        else:
            # For lifecycle nodes: map from state string to enum
            return get_node_status_states().get(self.state, NodeStatus.STATE_UNKNOWN)


def to_relative_time(current_time, past_time):
    """
    Format the time difference between current_time and past_time as a human-readable string.
    Example: "2 minutes ago", "5 seconds ago", etc.

    Args:
        current_time: Current ROS Time object
        past_time: Past ROS Time object

    Returns:
        str: Formatted relative time string
    """
    # Calculate time difference in seconds
    diff = current_time.nanoseconds - past_time.nanoseconds
    diff_sec = diff / 1e9

    # Format as relative time
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
    """
    Convert ROS Time to ISO8601 formatted timestamp string.

    Args:
        ros_time: ROS Time object

    Returns:
        str: ISO8601 formatted timestamp string
    """
    # Convert nanoseconds to seconds and fraction
    seconds = ros_time.nanoseconds // 1_000_000_000
    nanoseconds = ros_time.nanoseconds % 1_000_000_000

    # Create datetime from timestamp
    dt = datetime.fromtimestamp(seconds)

    # Format with microsecond precision (ISO8601)
    microseconds = nanoseconds // 1000
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


class NodeMonitor(Node):
    def __init__(self):
        super().__init__("node_lister")

        # Create callback groups for concurrent service calls
        self.state_cb_group = MutuallyExclusiveCallbackGroup()

        # Create publisher for node list
        self.publisher = self.create_publisher(NodeList, "/node_list", 10)

        # Dictionary of NodeInfo objects for each known node
        self.nodes: Dict[str, NodeInfo] = {}

        # Set to keep track of known lifecycle nodes
        self.lifecycle_nodes = set()

        # Subscribe to lifecycle transition events
        self.transition_event_sub = self.create_subscription(
            TransitionEvent,
            "/lifecycle_events",
            self.on_transition_event,
            10,
            callback_group=self.state_cb_group,
        )

        # Set up timer for periodic updates
        self.timer = self.create_timer(
            1.0, self.timer_callback, callback_group=self.state_cb_group  # 1 second
        )

        self.get_logger().info(
            "Node Monitor started, monitoring for lifecycle and regular nodes"
        )

    def on_transition_event(self, msg: TransitionEvent):
        """Handle lifecycle transition events from other nodes"""
        node_id = msg.id
        state_label = LIFECYCLE_STATES.get(msg.goal_state.id, "UNKNOWN")

        self.get_logger().info(
            f"Lifecycle event: {node_id} transitioning to {state_label}"
        )

        # Mark this as a lifecycle node
        self.lifecycle_nodes.add(node_id)

        # Update state if we're already tracking this node
        if node_id in self.nodes:
            self.nodes[node_id].is_lifecycle_node = True
            self.nodes[node_id].state = state_label
            self.nodes[node_id].last_seen = self.get_clock().now()

    def check_is_lifecycle_node(self, node_name: str) -> bool:
        """Check if a node is a lifecycle node"""
        # If we've seen transitions from this node, it's definitely a lifecycle node
        if node_name in self.lifecycle_nodes:
            return True

        # Check if node has get_state service (lifecycle nodes should have this)
        service_name = f"{node_name}/get_state"
        client = self.create_client(
            GetState, service_name, callback_group=self.state_cb_group
        )

        has_service = client.wait_for_service(timeout_sec=0.1)
        if has_service:
            self.lifecycle_nodes.add(node_name)  # Remember for next time
            return True

        return False

    def timer_callback(self):
        """Periodic callback to update node list"""
        # Get current time
        current_time = self.get_clock().now()

        # Get the list of currently active nodes
        current_node_names = set()
        for name, namespace in self.get_node_names_and_namespaces():
            # Form a fully qualified node name with namespace
            if namespace and namespace != "/":
                full_name = namespace.rstrip("/") + "/" + name
            else:
                full_name = "/" + name

            # Skip ourselves
            if full_name == self.get_fully_qualified_name():
                continue

            current_node_names.add(full_name)

            # Determine if this is a lifecycle node
            is_lifecycle = self.check_is_lifecycle_node(full_name)

            # Set state based on node type
            if is_lifecycle:
                # For lifecycle nodes: Try to get the actual state if not already known
                # If we can't get it, leave it as is or set to ACTIVE if new
                if full_name not in self.nodes:
                    # New lifecycle node, start with ACTIVE as default
                    node_state = "ACTIVE"
                else:
                    # Keep existing state for lifecycle nodes
                    node_state = self.nodes[full_name].state
            else:
                # For regular nodes: always ACTIVE if present
                node_state = "ACTIVE"

            # Update or create NodeInfo for this node
            if full_name in self.nodes:
                node_info = self.nodes[full_name]
                node_info.is_lifecycle_node = is_lifecycle
                node_info.state = node_state
                node_info.last_seen = current_time
                node_info.relative_time = "just now"
            else:
                self.nodes[full_name] = NodeInfo(
                    name=full_name,
                    last_seen=current_time,
                    is_lifecycle_node=is_lifecycle,
                    state=node_state,
                    relative_time="just now",
                )

        # Update status for nodes that disappeared
        for node_name, node_info in self.nodes.items():
            if node_name not in current_node_names:
                # Update state based on node type
                if node_info.is_lifecycle_node:
                    # For lifecycle nodes that were active but disappeared:
                    # Mark as FINALIZED if they were active/inactive/unconfigured
                    if node_info.state in ["ACTIVE", "INACTIVE", "UNCONFIGURED"]:
                        node_info.state = "FINALIZED"
                else:
                    # For regular nodes that disappeared: always INACTIVE
                    node_info.state = "INACTIVE"

                # Update the relative time for disappeared nodes
                node_info.relative_time = to_relative_time(
                    current_time, node_info.last_seen
                )

        # Prepare NodeStatus list for all known nodes
        node_status_list = []
        active_count = 0
        inactive_count = 0

        for node_name, node_info in self.nodes.items():
            # Count nodes
            if node_info.state == "ACTIVE":
                active_count += 1
            else:
                inactive_count += 1

            # Create a NodeStatus message
            status_msg = NodeStatus()
            status_msg.name = node_info.name

            # Use string representation of state
            status_msg.state = node_info.state

            # Use ISO8601 formatted timestamp
            status_msg.last_seen = to_iso8601(node_info.last_seen)

            # Set node type field
            status_msg.node_type = (
                "lifecycle" if node_info.is_lifecycle_node else "regular"
            )

            # Set relative time without redundant state and node type
            status_msg.relative_time = node_info.relative_time

            node_status_list.append(status_msg)

        # Create and populate the NodeList message
        node_list_msg = NodeList()
        node_list_msg.nodes = node_status_list
        node_list_msg.total_nodes = len(node_status_list)
        node_list_msg.active_nodes = active_count
        node_list_msg.inactive_nodes = inactive_count

        # Publish the NodeList message
        self.publisher.publish(node_list_msg)
        # Log the publication
        self.get_logger().info(
            f"Published /node_list with {node_list_msg.total_nodes} nodes "
            f"({node_list_msg.active_nodes} active, {node_list_msg.inactive_nodes} inactive)."
        )


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = NodeMonitor()

    # Use a MultiThreadedExecutor for handling service calls concurrently
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Run the node
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        executor.shutdown()
        rclpy.shutdown()
