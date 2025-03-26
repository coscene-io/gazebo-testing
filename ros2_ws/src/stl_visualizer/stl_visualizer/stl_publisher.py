import rclpy
from rclpy.node import Node
import numpy as np
from stl import mesh
from geometry_msgs.msg import Pose,TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster

class STLVisualizer(Node):
    def __init__(self):
        super().__init__('stl_visualizer')
        
        # 静态TF广播器
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # 设置静态TF参数（根据SDF中的初始pose）
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "map"
        static_transform.child_frame_id = "link_3"
        static_transform.transform.translation.x = 1.0    # X坐标
        static_transform.transform.translation.y = -0.5   # Y坐标
        static_transform.transform.translation.z = 2.65   # Z坐标
        static_transform.transform.rotation.w = 1.0       # 无旋转
        
        # 发布静态TF
        self.static_broadcaster.sendTransform(static_transform)

        # 加载STL文件
        self.mesh = mesh.Mesh.from_file('/input/models/RMUL2024_world/meshes/RMUL_2024.stl')
        
        # 创建发布者
        self.marker_pub = self.create_publisher(Marker, '/stl_marker', 10)
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时器
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        try:
            # 获取map到link_3的变换
            transform = self.tf_buffer.lookup_transform(
                'map',
                'link_3',
                rclpy.time.Time())
            
            # 创建Marker消息
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = "http://localhost:8000/RMUL_2024.stl"
            
            # 设置位姿和缩放
            marker.pose.position.x = transform.transform.translation.x
            marker.pose.position.y = transform.transform.translation.y
            marker.pose.position.z = transform.transform.translation.z
            marker.pose.orientation = transform.transform.rotation
            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            
            # 设置颜色
            marker.color.r = 0.7
            marker.color.g = 0.7
            marker.color.b = 0.7
            marker.color.a = 1.0
            
            self.marker_pub.publish(marker)
            
        except Exception as e:
            self.get_logger().warn(f'TF转换异常: {str(e)}')

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
