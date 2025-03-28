from geometry_msgs.msg import PoseWithCovarianceStamped


def load_pose(pose_data):
    if pose_data is None:
        pose_data = {
            "header": {"frame_id": "map"},
            "pose": {
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "covariance": [0.0] * 36,  # 示例协方差
            },
        }

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = pose_data["header"]["frame_id"]

    msg.pose.pose.position.x = pose_data["pose"]["pose"]["position"]["x"]
    msg.pose.pose.position.y = pose_data["pose"]["pose"]["position"]["y"]
    msg.pose.pose.position.z = pose_data["pose"]["pose"]["position"]["z"]

    msg.pose.pose.orientation.x = pose_data["pose"]["pose"]["orientation"]["x"]
    msg.pose.pose.orientation.y = pose_data["pose"]["pose"]["orientation"]["y"]
    msg.pose.pose.orientation.z = pose_data["pose"]["pose"]["orientation"]["z"]
    msg.pose.pose.orientation.w = pose_data["pose"]["pose"]["orientation"]["w"]

    msg.pose.covariance = pose_data["pose"]["covariance"]
    return msg
