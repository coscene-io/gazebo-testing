from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Transform, Vector3, Pose, Quaternion, TransformStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from tf2_ros import Buffer
from rclpy.time import Time
from urllib.parse import urljoin

@dataclass
class MarkerDescription:
    mesh_server_url: str
    # relative path to stl file in http server
    mesh_resource: str
    color: ColorRGBA
    scale: Vector3
    frame_id: str
    child_frame_id: str
    transform: Transform

    @classmethod
    def from_dict(cls, data: dict):
        try:
            print(data)
            return cls(
                mesh_server_url=data["mesh_server_url"],
                mesh_resource=data["mesh_resource"],
                color=ColorRGBA(**data["color"]),
                scale=Vector3(**data["scale"]),
                frame_id=data.get("frame_id"),
                child_frame_id=data.get("child_frame_id"),
                transform=cls._to_transform(data["transform"]),
            )
        except Exception as e:
            raise ValueError(f"Failed to parse marker description: {str(e)}\n {data}")

    def to_marker(self, tf_buffer: Buffer, stamp: Time) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp.to_msg()

        # Set mesh resource
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = urljoin(self.mesh_server_url, self.mesh_resource)

        # Set color and scale
        marker.color = self.color
        marker.scale = self.scale

        # Get transform from map to child_frame_id
        transform_stamped = tf_buffer.lookup_transform(
            self.frame_id,
            self.child_frame_id,
            stamp,
        )
        marker.pose = self._transform_to_pose(transform_stamped.transform)
        return marker

    @staticmethod
    def _to_transform(data: dict) -> Transform:
        transform = Transform()
        if "translation" in data:
            transform.translation = Vector3(**data["translation"])
        if "rotation" in data:
            transform.rotation = Quaternion(**data["rotation"])
        return transform
        
    @staticmethod
    def _transform_to_pose(transform: Transform) -> Pose:
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation.x = transform.rotation.x
        pose.orientation.y = transform.rotation.y
        pose.orientation.z = transform.rotation.z
        pose.orientation.w = transform.rotation.w
        return pose
