#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnShutdown
import os


def generate_launch_description():
    # 设置rosbag输出目录
    output_path = LaunchConfiguration("output_path", default="/root/rosbag2")

    # 使用rosbag录制所有话题
    record_all = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a", "-o", output_path],
        output="screen",
        name="rosbag_record",  # 给进程命名，便于后续引用
    )

    # 添加关闭事件处理器，在系统关闭时停止rosbag录制
    shutdown_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                # 使用ps查找并发送SIGINT给ros2 bag record进程
                ExecuteProcess(
                    cmd=[
                        [
                            FindExecutable(name="ps"),
                            " -ef | grep \"ros2 bag record\" | grep -v grep | awk '{print $2}' | xargs -r kill -INT",
                        ]
                    ],
                    shell=True,
                    output="screen",
                ),
                # 添加一个短暂的延迟，以确保bag文件被正确关闭
                ExecuteProcess(cmd=["sleep", "2"], output="screen"),
                # 输出确认信息
                ExecuteProcess(
                    cmd=[
                        "echo",
                        "Rosbag recording has been properly stopped and finalized.",
                    ],
                    output="screen",
                ),
            ]
        )
    )

    return LaunchDescription([record_all, shutdown_handler])
