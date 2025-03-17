#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String

class NavLoop(Node):
    def __init__(self):
        super().__init__('nav_loop_node')
        
        # 初始化导航Action客户端
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # 错误状态发布器
        self.error_pub = self.create_publisher(String, '/error_status', 10)
        
        # 初始位姿发布器（用于模拟定位丢失）
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )

        # 订阅错误话题
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String, '/error_status', self.error_callback, qos_profile)

        # 导航点配置
        self.targets = [
            {"x": 2.1, "y": 1.5, "w": 1.0},   # A点
            {"x": -0.8, "y": -1.7, "w": 1.0},  # B点
            {"x": -3.8, "y": 3.6, "w": 1.0},   # C点
            {"x": -3.2, "y": 6.8, "w": 1.0}    # D点
        ]
        
        # 状态跟踪变量
        self.current_target_index = 0    # 当前目标索引
        self.loop_count = 30             # 总循环次数
        self.current_loop = 0            # 当前已完成循环数
        self.stop_navigation = False     # 导航停止标志
        self.current_goal_handle = None  # 当前目标句柄
        self.pending_goal = False        # 目标发送状态
        self.recovery_timer = None       # 恢复定时器（新增）
        
        # 错误代码映射
        self.error_codes = {
            1001: "我的轮子坏了",
            1002: "我的激光坏了",
            1003: "我的摄像头坏了",
            1004: "有未知异常，请检查",
            1005: "我定位丢了！"
        }

        # 等待导航服务就绪
        self.client.wait_for_server()
        self.get_logger().info('导航服务已就绪')
        self.send_goal()

    def send_goal(self):
        """发送导航目标（带状态检查）"""
        # 检查停止状态和正在处理的目标
        if self.stop_navigation or self.pending_goal:
            self.get_logger().warn("当前处于暂停状态，不发送新目标")
            return
            
        # 检查总循环次数
        if self.current_loop >= self.loop_count:
            self.get_logger().info("已完成所有导航任务！")
            return

        # 设置目标发送状态
        self.pending_goal = True
        
        # 构建目标消息
        target = self.targets[self.current_target_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = target["x"]
        goal_msg.pose.pose.position.y = target["y"]
        goal_msg.pose.pose.orientation.w = target["w"]

        self.get_logger().info(f'开始导航至目标点: {target}')
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("目标被服务器拒绝")
            self.pending_goal = False
            return

        self.get_logger().info("目标已被接受，开始导航...")
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """处理导航结果"""
        self.current_goal_handle = None
        self.pending_goal = False

        # 检查是否处于停止状态
        if self.stop_navigation:
            self.get_logger().warn("因错误状态暂停导航")
            return

        self.get_logger().info("成功到达目标点")
        self.current_target_index += 1

        # 完成完整循环时触发异常
        if self.current_target_index >= len(self.targets):
            self.current_target_index = 0
            self.current_loop += 1
            self.get_logger().warning(f"完成第{self.current_loop}次循环，触发异常测试")
            self.trigger_random_error()
        else:
            self.send_goal()

    def trigger_random_error(self):
        """触发随机错误（带定时器管理）"""
        # 取消之前的恢复定时器
        if self.recovery_timer and not self.recovery_timer.is_canceled():
            self.recovery_timer.cancel()
            self.recovery_timer = None

        # 随机选择错误代码
        error_code = random.choice(list(self.error_codes.keys()))
        error_msg = f"errorcode:{error_code},info:{self.error_codes[error_code]},value:1"
        
        # 发布错误状态
        self.error_pub.publish(String(data=error_msg))
        self.get_logger().error(f"⚠️ ！触发严重异常: {error_msg}")

        # 立即停止当前导航
        self.stop_navigation = True
        self.cancel_current_goal()

        # 处理不同类型错误
        if error_code != 1005:
            # 设置单次触发定时器（关键修改）
            self.recovery_timer = self.create_timer(
                15.0, 
                lambda: self.clear_error(error_code),
                oneshot=True
            )
        else:
            self.simulate_localization_jump()

    def cancel_current_goal(self):
        """取消当前导航目标"""
        if self.current_goal_handle is not None:
            self.get_logger().info("正在取消当前导航目标...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """处理取消操作结果"""
        self.get_logger().info("成功取消导航目标")
        self.current_goal_handle = None

    def clear_error(self, error_code):
        """清除错误状态（带安全检查）"""
        # 取消可能存在的定时器
        if self.recovery_timer and not self.recovery_timer.is_canceled():
            self.recovery_timer.cancel()
            self.recovery_timer = None

        # 仅当处于错误状态时处理
        if self.stop_navigation:
            self.get_logger().info(f"正在清除错误码 {error_code}...")
            
            # 发布清除消息
            clear_msg = f"errorcode:{error_code},info:错误已恢复,value:0"
            self.error_pub.publish(String(data=clear_msg))
            self.get_logger().info(f"🔄 成功清除错误: {clear_msg}")
            
            # 更新状态
            self.stop_navigation = False
            
            # 安全恢复导航
            if not self.pending_goal and self.current_goal_handle is None:
                self.send_goal()

    def simulate_localization_jump(self):
        """模拟定位丢失（带随机坐标跳变）"""
        self.get_logger().error("‼️ 正在模拟定位系统故障...")
        
        # 生成随机坐标
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = random.uniform(-5.0, 5.0)
        pose_msg.pose.pose.position.y = random.uniform(-5.0, 5.0)
        pose_msg.pose.pose.orientation.w = 1.0
        
        # 发布假定位数据
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().error(
            f"‼️ 已模拟定位跳变至坐标: ({pose_msg.pose.pose.position.x:.1f}, " +
            f"{pose_msg.pose.pose.position.y:.1f})"
        )

    def error_callback(self, msg):
        """处理错误状态消息"""
        try:
            # 解析消息内容
            error_dict = dict(part.split(':') for part in msg.data.split(','))
            error_code = int(error_dict.get("errorcode", "0"))
            error_value = error_dict.get("value", "0")
        except Exception as e:
            self.get_logger().error(f"错误消息解析失败: {str(e)}")
            return

        # 根据错误值处理
        if error_value == "1":
            self.handle_error_occurred(error_code)
        elif error_value == "0":
            self.handle_error_resolved()

    def handle_error_occurred(self, error_code):
        """处理错误发生事件"""
        # 定位丢失特殊处理
        if error_code == 1005:
            self.get_logger().error("‼️ 需要人工干预的定位丢失！")
            self.stop_navigation = True
            self.cancel_current_goal()
        else:
            # 其他错误处理
            if not self.stop_navigation:
                self.get_logger().error(f"检测到系统异常（代码 {error_code}），正在停止...")
            self.stop_navigation = True
            self.cancel_current_goal()

    def handle_error_resolved(self):
        """处理错误解除事件"""
        if self.stop_navigation:
            self.get_logger().info("系统已恢复正常运行状态")
        self.stop_navigation = False
        
        # 安全恢复导航
        if not self.pending_goal and self.current_goal_handle is None:
            self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    navigator = NavLoop()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("用户终止导航任务")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
