#!/usr/bin/env python3
import rclpy
import os
import json
import yaml
import random
import sys
from uuid import uuid4
from datetime import datetime
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus  # 确保正确导入状态码

class TaskManager:
    def __init__(self, config_path):
        self.config_path = config_path
        self.all_points = []
        self.task_queue = []
        self.history = []
        self.load_config()
        
    def load_config(self):
        try:
            with open(self.config_path, 'r') as f:
                data = yaml.safe_load(f)
                self.all_points = data['points']
        except Exception as e:
            raise RuntimeError(f"配置文件加载失败: {str(e)}")
            
    def generate_task(self, num=10):
        if len(self.all_points) < num:
            raise ValueError(f"配置文件仅含{len(self.all_points)}个点，需要至少{num}个点")
            
        selected = random.sample(self.all_points, num)
        self.task_queue = [
            {
                'id': f"TASK-{uuid4().hex[:6]}",
                'point': p,
                'status': 'pending',
                'start_time': None,
                'end_time': None,
                'error_info': ''
            } for p in selected
        ]
    
    def get_next_task(self):
        return self.task_queue[0] if self.task_queue else None
    
    def mark_task_complete(self, task_id, success=True, error_info=""):
        for idx, task in enumerate(self.task_queue):
            if task['id'] == task_id:
                task['status'] = 'success' if success else 'failed'
                task['end_time'] = datetime.now()
                task['error_info'] = error_info
                completed = self.task_queue.pop(idx)
                self.history.append(completed)
                return completed
        return None

class NavController(Node):
    def __init__(self):
        super().__init__('advanced_nav_controller')
        
        # 参数声明
        self.declare_parameter('config_path', '')
        self.declare_parameter('report_path', '/home/qingyu')
        
        try:
            config_path = self.get_parameter('config_path').value
            if not os.path.exists(config_path):
                raise FileNotFoundError(f"配置文件不存在: {config_path}")
                
            self.task_mgr = TaskManager(config_path)
            self.task_mgr.generate_task()
            
        except Exception as e:
            self.get_logger().fatal(f"初始化失败: {str(e)}")
            raise

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.task_success = 0
        self.task_fail = 0
        
        qos = QoSProfile(depth=10)
        self.sub_error = self.create_subscription(
            String, '/error_status', self.error_callback, qos)
        self.pub_error = self.create_publisher(String, '/error_status', qos)
        
        self.current_goal_handle = None
        self.stop_navigation = False
        self.current_task = None
        
        self.nav_client.wait_for_server()
        self.start_next_task()

    def start_next_task(self):
        """启动下一任务"""
        if self.stop_navigation:
            return

        next_task = self.task_mgr.get_next_task()
        if not next_task:
            self.generate_report()
            self.destroy_node()
            rclpy.shutdown()
            return

        self.current_task = next_task
        self.current_task['start_time'] = datetime.now()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.current_task['point']['x']
        goal_msg.pose.pose.position.y = self.current_task['point']['y']
        goal_msg.pose.pose.orientation.w = self.current_task['point'].get('w', 1.0)
        
        self.get_logger().info(f"开始任务 {self.current_task['id']}")
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.handle_task_failure("目标被服务器拒绝")
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """关键修复：使用状态码而非结果字段"""
        try:
            result_wrapper = future.result()
            
            # 获取动作最终状态
            goal_status = result_wrapper.status
            
            # 状态码映射
            status_map = {
                GoalStatus.STATUS_UNKNOWN: "未知状态",
                GoalStatus.STATUS_ACCEPTED: "已接受",
                GoalStatus.STATUS_EXECUTING: "执行中",
                GoalStatus.STATUS_CANCELING: "取消中",
                GoalStatus.STATUS_SUCCEEDED: "成功",
                GoalStatus.STATUS_CANCELED: "已取消",
                GoalStatus.STATUS_ABORTED: "已放弃"
            }
            
            self.get_logger().info(f"导航最终状态: {status_map.get(goal_status, '未知状态')} ({goal_status})")

            if goal_status == GoalStatus.STATUS_SUCCEEDED:
                self.handle_task_success()
            else:
                self.handle_task_failure(status_map[goal_status])

        except Exception as e:
            error_msg = f"结果处理异常: {str(e)}"
            self.get_logger().error(error_msg)
            self.handle_task_failure(error_msg)

    def handle_task_success(self):
        self.task_mgr.mark_task_complete(self.current_task['id'])
        self.task_success += 1
        self.get_logger().info(f"任务 {self.current_task['id']} 成功完成")
        self.start_next_task()

    def handle_task_failure(self, reason):
        error_msg = String()
        error_msg.data = f"code:10001,info:{reason},value:1"
        self.pub_error.publish(error_msg)
        
        self.task_mgr.mark_task_complete(
            self.current_task['id'], 
            success=False, 
            error_info=reason
        )
        self.task_fail += 1
        self.get_logger().error(f"任务失败: {reason}")
        self.start_next_task()

    def error_callback(self, msg):
        error_dict = {}
        try:
            for item in msg.data.split(','):
                if ':' not in item:
                    continue
                key, value = item.split(':', 1)
                error_dict[key.strip()] = value.strip()
            
            code = int(error_dict.get('code', 0))
            value = error_dict.get('value', '0')

            if 1000 <= code <= 1999:
                if value == '1':
                    self.emergency_stop()
                elif value == '0' and self.stop_navigation:
                    self.resume_navigation()
                
        except Exception as e:
            self.get_logger().error(f"错误解析失败: {str(e)} 原始数据：{msg.data}")

    def emergency_stop(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.stop_navigation = True
        self.get_logger().warn("! 紧急停止激活 !")

    def resume_navigation(self):
        self.stop_navigation = False
        self.get_logger().info("系统恢复正常运行")
        if not self.current_goal_handle:
            self.start_next_task()

    def generate_report(self):
        """关键修复：路径自动创建"""
        try:
            report_path = os.path.join(
                self.get_parameter('report_path').value,
                f"nav_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
            
            # 确保目录存在
            report_dir = os.path.dirname(report_path)
            os.makedirs(report_dir, exist_ok=True)
            
            report = {
                "metadata": {
                    "report_id": f"NAV-REPORT-{datetime.now().strftime('%Y%m%d%H%M%S')}",
                    "generation_time": datetime.now().isoformat(),
                    "total_tasks": len(self.task_mgr.history),
                    "success_count": self.task_success,
                    "failure_count": self.task_fail
                },
                "tasks": [
                    {
                        "task_id": t['id'],
                        "coordinates": {
                            "x": t['point']['x'], 
                            "y": t['point']['y'],
                            "orientation": t['point'].get('w', 1.0)
                        },
                        "status": t['status'],
                        "duration_sec": (t['end_time'] - t['start_time']).total_seconds() if t['end_time'] else 0,
                        "error_info": t['error_info']
                    } for t in self.task_mgr.history
                ]
            }

            with open(report_path, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"测试报告已生成: {report_path}")
            
        except PermissionError:
            self.get_logger().error(f"无权限写入文件: {report_path}")
        except Exception as e:
            self.get_logger().error(f"生成报告失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    controller = None
    
    try:
        controller = NavController()
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        if controller:
            controller.get_logger().info("用户终止操作")
        else:
            rclpy.logging.get_logger('nav_controller').warning("启动前被中断")
            
    except Exception as e:
        if controller:
            controller.get_logger().error(f"运行时异常: {str(e)}")
        else:
            error_msg = f"启动失败: {str(e)}"
            rclpy.logging.get_logger('nav_controller').fatal(error_msg)
            print(f"\033[31mFATAL: {error_msg}\033[0m", file=sys.stderr)
            
    finally:
        if controller is not None:
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
