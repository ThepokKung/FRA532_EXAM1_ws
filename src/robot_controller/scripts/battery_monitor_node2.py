#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time
from std_srvs.srv import SetBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robot_interfaces.srv import RobotStateUpdate, RobotStateCheck, RobotStationCheck, RobotStationUpdate,Station2GO
class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        # Battery level subscriber
        self.battery_level_monitor = self.create_subscription(Float32, 'battery_level', self.battery_status_callback, 10)

        self.station2go_client_cbg = MutuallyExclusiveCallbackGroup()
        self.station2go_client = self.create_client(Station2GO,'/station_2go',callback_group=self.station2go_client_cbg)

        # Service client for starting charging
        self.charger_call_client_cbg = MutuallyExclusiveCallbackGroup()
        self.charger_call_client = self.create_client(SetBool, 'charger_call', callback_group=self.charger_call_client_cbg)

        # Robot state and station services
        self.robot_station_check_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_station_update_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_state_check_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_state_update_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_station_check = self.create_client(RobotStationCheck, '/check_robot_station', callback_group=self.robot_station_check_cbg)
        self.robot_station_update = self.create_client(RobotStationUpdate, '/update_robot_station',callback_group=self.robot_station_update_cbg)
        self.robot_state_check = self.create_client(RobotStateCheck, '/check_robot_state',callback_group=self.robot_state_check_cbg)
        self.robot_state_update = self.create_client(RobotStateUpdate, '/update_robot_state',callback_group=self.robot_state_update_cbg)

        # Wait for required services
        self.wait_for_services()
        self.get_logger().info('Battery monitor node has been started.')

    def update_robot_state(self, state):
        """Update robot state via service."""
        request = RobotStateUpdate.Request()
        request.state = state
        future = self.robot_state_update.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def update_robot_station(self, station):
        """Update robot station via service."""
        request = RobotStationUpdate.Request()
        request.station = station
        future = self.robot_station_update.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None
    
    def check_robot_station(self):
        """Call the /check_robot_station service to set the current station's ArUco ID."""
        request = RobotStationCheck.Request()
        request.checkstation = True
        future = self.robot_station_update.call_async(request)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            # self.get_logger().info(f"Set Station to {station} successfully.")
            return future.result().station
        else:
            self.get_logger().error("Failed to call /update_robot_station service.")
            return False
        
    def check_robot_state(self):
        """Call the /check_robot_state service to get the current robot state."""
        request = RobotStateCheck.Request()
        request.checkstate = True
        future = self.robot_state_check.call_async(request)

        while not future.done():  # Avoid blocking the service
            time.sleep(0.1)

        if future.result() is not None:
            self.get_logger().info(f"Robot is currently at: {future.result().state}")
            return future.result().state
        else:
            self.get_logger().error("Failed to call /check_robot_state service.")
            return None


    def wait_for_services(self):
        """Wait for necessary services to be available."""
        services = [
            (self.charger_call_client, 'charger_call'),
            (self.robot_state_update, 'update_robot_state'),
            (self.robot_state_check, 'check_robot_state'),
            (self.robot_station_update, 'update_robot_station'),
            (self.robot_station_check, 'check_robot_station')
        ]
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service_name} service...')

    
    def check_robot_state_we_can_go(self):
        """Check if robot state allows navigation."""
        robot_state = self.check_robot_state()
        if robot_state and robot_state.strip() == "None":
            self.get_logger().info("Robot state is 'None'. Navigation allowed.")
            
            return True
        else:
            self.get_logger().error(f"Robot state is '{robot_state}', not 'None'. Navigation denied.")
            return False

    def go_to_charge_station(self):
        """Navigate to the charging station."""
        msg_request = Station2GO.Request()  
        msg_request.station = 'ChangeStation'
        self.station2go_client.call_async(msg_request)

    def battery_status_callback(self, msg):
        """Check battery status and take action."""
        battery_level = msg.data
        if battery_level < 20:
            self.get_logger().info('Battery level is low. Navigating to charger.')
            self.check_robot_state_we_can_go()
        else:
            self.get_logger().info('Battery level is normal. No action needed.')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


