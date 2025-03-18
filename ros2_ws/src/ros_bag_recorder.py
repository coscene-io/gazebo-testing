#!/usr/bin/env python3
import subprocess
import os
import time

# 录制文件根目录
RECORD_DIR = "/home/qingyu/turtlebot3_ws_my/bag"
os.makedirs(RECORD_DIR, exist_ok=True)
# 切换工作目录到 RECORD_DIR，让 ros2 bag record 自动在此目录下创建子目录
os.chdir(RECORD_DIR)

# 需要录制的所有 topics 列表
topics = [
    "/amcl_pose",
    "/cmd_vel_nav",
    "/error_status",
    "/global_costmap/costmap",
    "/global_costmap/costmap_updates",
    "/global_costmap/footprint",
    "/global_costmap/published_footprint",
    "/goal_pose",
    "/local_costmap/costmap",
    "/local_costmap/costmap_updates",
    "/local_costmap/footprint",
    "/local_costmap/published_footprint",
    "/local_plan",
    "/map",
    "/odom",
    "/plan",
    "/plan_smoothed",
    "/received_global_plan",
    "/robot_description",
    "/scan",
    "/tf",
    "/tf_static",
    "/rosout"
]

# 单个 mcap 文件的大小上限，这里设置为 200M（单位：字节）
max_split_size = "10071520"

def main():
    while True:
        print("启动 ros2 bag 录制进程")
        # 构造 ros2 bag record 命令，不传 --output 参数，ros2 bag 会在 RECORD_DIR 下自动创建子目录
        cmd = [
            "ros2", "bag", "record",
            "-s", "mcap",
            "-b", max_split_size
        ] + topics

        try:
            proc = subprocess.Popen(cmd)
            proc.wait()
        except KeyboardInterrupt:
            print("接收到 KeyboardInterrupt，终止录制进程")
            proc.terminate()
            break
        except Exception as e:
            print("录制进程出现异常:", e)

        # 如果录制进程意外退出，等待 2 秒后重新启动
        time.sleep(2)

if __name__ == '__main__':
    main()
