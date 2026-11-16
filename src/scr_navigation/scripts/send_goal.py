#!/usr/bin/env python3
"""
简单的导航目标发送脚本
用法: ros2 run scr_navigation send_goal.py x y yaw
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self._action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )
        
    def send_goal(self, x, y, yaw=0.0):
        """发送导航目标"""
        self.get_logger().info(f'等待导航服务器...')
        self._action_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # 将yaw转换为四元数
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'发送目标: x={x}, y={y}, yaw={yaw}')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False
        
        self.get_logger().info('目标已接受，等待结果...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✅ 导航成功！')
            return True
        else:
            self.get_logger().error(f'❌ 导航失败，状态: {result.status}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print('用法: ros2 run scr_navigation send_goal.py <x> <y> [yaw]')
        print('示例: ros2 run scr_navigation send_goal.py 2.0 1.0 0.0')
        sys.exit(1)
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    navigator = SimpleNavigator()
    navigator.send_goal(x, y, yaw)
    
    navigator.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
