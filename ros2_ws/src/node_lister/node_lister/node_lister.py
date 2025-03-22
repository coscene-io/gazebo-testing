import rclpy
from node_msgs.msg import NodeList, NodeStatus
from rclpy.node import Node
from rclpy.time import Time


class NodeMonitor(Node):
    def __init__(self):
        super().__init__("node_lister")
        # Publisher for NodeList messages
        self.publisher = self.create_publisher(NodeList, "/node_list", 10)
        # Timer to periodically update (default: 1 second interval)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # Dictionary to store the last seen time for each node (by name)
        self.last_seen = {}
        self.get_logger().info("Node Monitor started, publishing to /node_list")

    def timer_callback(self):
        # Get the list of currently active nodes (names with namespace if available)
        current_nodes = []
        for name, namespace in self.get_node_names_and_namespaces():
            # Form a fully qualified node name with namespace (avoid trailing '//')
            if namespace and namespace != "/":
                full_name = namespace.rstrip("/") + "/" + name
            else:
                full_name = name
            current_nodes.append(full_name)
            # Update last seen time for this node to now
            self.last_seen[full_name] = self.get_clock().now()

        # Prepare NodeStatus list for all known nodes
        node_status_list = []
        active_count = 0
        inactive_count = 0
        for node_name, last_time in self.last_seen.items():
            is_active = node_name in current_nodes
            if is_active:
                active_count += 1
            else:
                inactive_count += 1
            # Create a NodeStatus message for each known node
            status_msg = NodeStatus()
            status_msg.name = node_name
            status_msg.active = is_active
            status_msg.last_seen = (
                last_time.to_msg()
            )  # convert Time to builtin_interfaces/Time message
            node_status_list.append(status_msg)

        # Create and populate the NodeList message
        node_list_msg = NodeList()
        node_list_msg.nodes = node_status_list
        node_list_msg.total_nodes = len(node_status_list)
        node_list_msg.active_nodes = active_count
        node_list_msg.inactive_nodes = inactive_count

        # Publish the NodeList message
        self.publisher.publish(node_list_msg)
        # Log the publication (for debugging/demo purposes)
        self.get_logger().info(
            f"Published /node_list with {node_list_msg.total_nodes} nodes "
            f"({node_list_msg.active_nodes} active, {node_list_msg.inactive_nodes} inactive)."
        )


def main(args=None):
    rclpy.init(args=args)
    node = NodeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly (optional - for clean shutdown)
        node.destroy_node()
        rclpy.shutdown()
