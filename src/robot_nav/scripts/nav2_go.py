#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from robot_interfaces.srv import Target2GO

class Nav2Go(Node):
    def __init__(self):
        super().__init__('nav2_go')
        
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.srv = self.create_service(Target2GO, 'target_to_go', self.target_to_go_callback)
        
        self.get_logger().info('Nav2Go node has been started.')

    def target_to_go_callback(self, request, response):
        self.get_logger().info(
            f'Received target coordinates: x={request.target.x}, y={request.target.y}, z={request.target.z}')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = request.target.x
        goal_msg.pose.pose.position.y = request.target.y
        goal_msg.pose.pose.orientation.z = request.target.z
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        response.success = True
        response.message = 'Target received successfully'
        return response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result: {result}')
        # Handle the result as needed

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Go()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
