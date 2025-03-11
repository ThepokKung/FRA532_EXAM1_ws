#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from robot_interfaces.srv import BatteryUpdate

class ChargeStationNode(Node):
    def __init__(self):
        super().__init__('ChargeStationNode')

        #Create server server
        self.charger_call = self.create_service(SetBool, 'charger_call', self.charger_callback)

        # Create a publisher for battery level updates
        self.battery_level_update = self.create_client(BatteryUpdate, 'battery_level_update')

        self.get_logger().info('ChargeStationNode has been started.')

    def charger_callback(self, request, response):
        if request.data:
            # Call the battery level update service
            battery_level_msg = BatteryUpdate.Request()
            battery_level_msg.battery_level = 100  
            self.battery_level_update.call_async(battery_level_msg)

            self.get_logger().info('Charging the robot.')
            response.success = True
            response.message = 'Charging the robot.'
        else:
            self.get_logger().info('Stop charging the robot.')
            response.success = True
            response.message = 'Stop charging the robot.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ChargeStationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()