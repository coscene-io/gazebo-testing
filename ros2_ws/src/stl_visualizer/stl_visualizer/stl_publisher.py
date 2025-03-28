import rclpy
from rclpy.node import Node
import yaml
import os
import numpy as np
from stl import mesh
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster

CONFIG_PATH = "/input/config/model.yaml"

class STLVisualizer(Node):
    def __init__(self):
        super().__init__('stl_visualizer')

        self.static_broadcaster = StaticTransformBroadcaster(self)

        # 读取 YAML 文件中的 TF 变换参数
        transform_params = self.load_transform_params(CONFIG_PATH, "link_3")

        # 定义静态 TF 变换
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = transform_params["frame_id"]
        static_transform.child_frame_id = transform_params["child_frame_id"]
        static_transform.transform.translation.x = transform_params["translation"]["x"]
        static_transform.transform.translation.y = transform_params["translation"]["y"]
        static_transform.transform.translation.z = transform_params["translation"]["z"]
        static_transform.transform.rotation.x = transform_params["rotation"]["x"]
        static_transform.transform.rotation.y = transform_params["rotation"]["y"]
        static_transform.transform.rotation.z = transform_params["rotation"]["z"]
        static_transform.transform.rotation.w = transform_params["rotation"]["w"]

        self.static_broadcaster.sendTransform(static_transform)

        self.mesh = mesh.Mesh.from_file('/input/models/RMUL2024_world/meshes/RMUL_2024.stl')

        self.marker_pub = self.create_publisher(Marker, '/stl_marker', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.publish_marker)

    def load_transform_params(self, file_path, target_child_frame):
        """从 YAML 文件加载 TF 变换参数"""
        default_params = {
            "frame_id": "map",
            "child_frame_id": target_child_frame,
            "translation": {"x": 6.53, "y": 2.28, "z": 1.5},
            "rotation": {"x": 0.3090, "y": 0.9511, "z": 0.0, "w": 0.0}
        }

        if os.path.exists(file_path):
            try:
                with open(file_path, "r") as file:
                    data = yaml.safe_load(file)
                    if data and "transforms" in data:
                        for transform in data["transforms"]:
                            if transform["child_frame_id"] == target_child_frame:
                                self.get_logger().info(
                                    f"load YAML succees, publish:\n"
                                    f"frame_id: {transform['frame_id']}, child_frame_id: {transform['child_frame_id']}\n"
                                    f"translation: x={transform['translation']['x']}, y={transform['translation']['y']}, z={transform['translation']['z']}\n"
                                    f"rotation: x={transform['rotation']['x']}, y={transform['rotation']['y']}, z={transform['rotation']['z']}, w={transform['rotation']['w']}"
                                )
                                return transform
            except Exception as e:
                self.get_logger().warn(f"加载 YAML 失败: {e}，使用默认参数")

        return default_params

    def publish_marker(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'link_3', rclpy.time.Time())

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = "http://localhost:8000/RMUL_2024.stl"
            marker.pose.position.x = transform.transform.translation.x
            marker.pose.position.y = transform.transform.translation.y
            marker.pose.position.z = transform.transform.translation.z
            marker.pose.orientation = transform.transform.rotation
            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            marker.color.r = 0.7
            marker.color.g = 0.7
            marker.color.b = 0.7
            marker.color.a = 1.0

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f'TF error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = STLVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
