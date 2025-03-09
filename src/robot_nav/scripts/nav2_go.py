#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from robot_interfaces.srv import Target2GO
from std_srvs.srv import SetBool

class Nav2Go(Node):
    def __init__(self):
        super().__init__('nav2_go')
        
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.srv = self.create_service(Target2GO, 'target_to_go', self.target_to_go_callback)

        self.docking_client = self.create_client(SetBool, '/toggle_docking')

        self.get_logger().info('✅ Nav2Go node has been started.')

    def target_to_go_callback(self, request, response):
        """Callback เมื่อได้รับเป้าหมายจาก Target2GO"""
        self.get_logger().info(f'📍 Received target: x={request.target.x}, y={request.target.y}, yaw={request.target.z}')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = request.target.x
        goal_msg.pose.pose.position.y = request.target.y
        goal_msg.pose.pose.orientation.z = request.target.z
        goal_msg.pose.pose.orientation.w = 1.0

        # รอให้ Nav2 Action Server พร้อม
        self._action_client.wait_for_server()
        
        # ส่งคำสั่งไปยัง Nav2
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        future_response = Future()
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, response, future_response))

        # รอให้ Action เสร็จสิ้น
        rclpy.spin_until_future_complete(self, future_response)
        return response

    def goal_response_callback(self, future, response, future_response):
        """Callback เมื่อ Nav2 ตอบรับ Goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            response.success = False
            response.message = 'Goal rejected'
            future_response.set_result(None)  # ปิด Future
            return

        self.get_logger().info('🚀 Goal accepted, navigating...')

        # รอให้ Nav2 ทำงานเสร็จ
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.get_result_callback(future, response, future_response))

    def get_result_callback(self, future, response, future_response):
        """Callback เมื่อหุ่นยนต์ไปถึงเป้าหมาย"""
        result = future.result().result
        self.get_logger().info(f'✅ Goal reached: {result}')

        # เปิด Docking Mode
        self.activate_docking_mode()

        # ส่งผลลัพธ์กลับไปยัง Client
        response.success = True
        response.message = 'Goal reached successfully'
        future_response.set_result(None)  # ปิด Future

    def activate_docking_mode(self):
        """สั่งเปิด Docking Mode"""
        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for toggle_docking service...')

        docking_request = SetBool.Request()
        docking_request.data = True

        future = self.docking_client.call_async(docking_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("✅ Docking mode activated successfully")
        else:
            self.get_logger().error("❌ Docking mode activation failed")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Go()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
