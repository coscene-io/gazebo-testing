import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker
import yaml

from marker_publisher.markers import MarkerDescription
from marker_publisher.mesh_server import MeshServer


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")

        self.declare_parameter("mesh_resource_dir", "")
        self.declare_parameter("mesh_server_url", "http://localhost:8000")
        self.declare_parameter("mesh_server_port", 8000)
        self.declare_parameter(
            "marker", "",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("marker_topic", "/visualization_marker")
        self.declare_parameter("publish_interval", 1.0)

        # Get parameter values
        self.mesh_resource_dir = self.get_parameter("mesh_resource_dir").value
        if not self.mesh_resource_dir:
            self.get_logger().error("mesh_resource_dir is not set, node will now shutdown")
            return

        self.mesh_server_port = self.get_parameter("mesh_server_port").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.publish_interval = self.get_parameter("publish_interval").value
        
        marker_value = self.get_parameter("marker").value
        if not marker_value:
            self.get_logger().info("No marker descriptions found, node will now shutdown")
            return
        try:
            data = yaml.safe_load(marker_value)
            data["mesh_server_url"] = self.get_parameter("mesh_server_url").value or f"http://localhost:{self.mesh_server_port}"
            self.desc = MarkerDescription.from_dict(data)
        except Exception as e:
            self.get_logger().error(f"Failed to load marker descriptions: {str(e)}")
            raise e 

        # Start HTTP server
        self.mesh_server = MeshServer(
            self,
            self.mesh_resource_dir,
            self.mesh_server_port
        )
        self.mesh_server.start()

        # Static TF broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf(self.desc)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publish marker periodically
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.timer = self.create_timer(self.publish_interval, self.publish_marker)


    def publish_static_tf(self, desc: MarkerDescription):
        # Set static TF parameters (based on initial pose in SDF)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = desc.frame_id
        static_tf.child_frame_id = desc.child_frame_id
        static_tf.transform = desc.transform

        # Publish static TF
        self.static_broadcaster.sendTransform(static_tf)
    
    def publish_marker(self):
        current_time = self.get_clock().now()
        try:
            marker = self.desc.to_marker(self.tf_buffer, current_time)
            self.marker_pub.publish(marker)
        except Exception as e:
            self.get_logger().warn(f"TF transform error: {str(e)}")

    def destroy_node(self):
        if hasattr(self, 'mesh_server'):
            self.mesh_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
