#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from std_srvs.srv import SetBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robot_interfaces.srv import RobotStateCheck, RobotStationCheck, Station2GO

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        # Battery level subscriber
        self.battery_level = 100  # Default high battery level
        self.battery_level_monitor = self.create_subscription(Float32, 'battery_level', self.battery_status_callback, 10)

        # Service clients
        self.station2go_client_cbg = MutuallyExclusiveCallbackGroup()
        self.charger_call_client_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_state_check_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_station_check_cbg = MutuallyExclusiveCallbackGroup()
        self.station2go_client = self.create_client(Station2GO, '/station_2go', callback_group=self.station2go_client_cbg)
        self.charger_call_client = self.create_client(SetBool, 'charger_call', callback_group=self.charger_call_client_cbg)
        self.robot_station_check = self.create_client(RobotStationCheck, '/check_robot_station', callback_group=self.robot_station_check_cbg)
        self.robot_state_check = self.create_client(RobotStateCheck, '/check_robot_state', callback_group=self.robot_state_check_cbg)

        # Timer to check battery every 10 seconds
        self.battery_check_timer = self.create_timer(5.0, self.handle_low_battery)

        # Flag to prevent multiple calls to go_to_charge_station()
        self.navigating_to_station = False

        self.get_logger().info('Battery monitor node has been started.')

    def battery_status_callback(self, msg):
        """Update battery level when a new message arrives."""
        self.battery_level = msg.data

    def handle_low_battery(self):
        """Check battery status periodically and take action if needed."""
        if self.battery_level < 20:
            self.get_logger().info('Battery level is low. Checking robot state...')
            if not self.navigating_to_station and self.check_robot_state_we_can_go():
                self.get_logger().info("Navigating to charge station.")
                self.navigating_to_station = True  # Set flag to prevent multiple calls
                self.go_to_charge_station()
                self.wait_until_on_charge_station()
                self.start_charging()
        else:
            self.get_logger().info('Battery level is normal. No action needed.')

    def check_robot_state_we_can_go(self):
        """Check if robot state allows navigation."""
        request = RobotStateCheck.Request()
        request.checkstate = True
        future = self.robot_state_check.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().state.strip() == "None":
            self.get_logger().info("Robot state is 'None'. Navigation allowed.")
            return True
        else:
            self.get_logger().error(f"Robot state is '{future.result().state}', not 'None'. Navigation denied.")
            return False

    def go_to_charge_station(self):
        """Navigate to the charging station."""
        request = Station2GO.Request()
        request.station = 'ChangeStation'
        self.station2go_client.call_async(request)

    def wait_until_on_charge_station(self):
        """Wait until the robot is at the charging station."""
        while True:
            request = RobotStationCheck.Request()
            request.checkstation = True
            future = self.robot_station_check.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().station == 'ChangeStation':
                self.get_logger().info("Robot has arrived at ChangeStation.")
                self.navigating_to_station = False  # Reset flag once reached
                break
            time.sleep(2)  # Check every 5 seconds

    def start_charging(self):
        """Call charger service to start charging."""
        request = SetBool.Request()
        request.data = True
        self.charger_call_client.call_async(request)
        self.get_logger().info("Started charging.")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
