#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from robot_interfaces.srv import BatteryUpdate

class BatterySimulatorNode(Node):
    def __init__(self):
        super().__init__('battery_simulator_node')

        self.publisher_ = self.create_publisher(Float32, 'battery_level', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_level)
        self.battery_level = 20.2

        self.battery_level_update = self.create_service(BatteryUpdate, 'battery_level_update', self.battery_level_update_callback)

        self.get_logger().info('Battery Simulator Node has been started.')

    def battery_level_update_callback(self, request, response):
        self.battery_level = request.battery_level
        response.success = True
        response.message = 'Battery level updated.'
        self.get_logger().info(f'Battery level updated to {self.battery_level:.2f}%)')
        return response

    def publish_battery_level(self):
        msg = Float32()
        self.battery_level -= 0.01  # Simulate battery drain
        if self.battery_level < 0:
            self.battery_level = 0
        msg.data = self.battery_level
        self.publisher_.publish(msg)
        self.get_logger().info(f'Battery level: {self.battery_level:.2f}%')

def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()