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

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        # Load charging station info
        self.station_path = 'robot_controller'
        path_file = os.path.join(get_package_share_directory(self.station_path), 'config', 'station.yaml')
        self.charger_station = self.load_station(path_file)

        # Battery level subscriber
        self.battery_level_monitor = self.create_subscription(Float32, 'battery_level', self.battery_status_callback, 10)

        # Action client for navigation
        self.navigate_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Service client for starting charging
        self.charger_call_client = self.create_client(SetBool, 'charger_call')

        # Wait for required services
        self.wait_for_services()

        self.get_logger().info('Battery monitor node has been started.')

    def load_station(self, filepath):
        """Load the charging station from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                for station in data:
                    if station['name'] == 'ChangeStation':
                        self.get_logger().info(f"Loaded charging station: {station['name']}")
                        return station  # Return the first matching station
            self.get_logger().error("No 'ChangeStation' found in station file.")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load station file: {e}")
            return None

    def wait_for_services(self):
        """Wait for necessary services to be available."""
        services = [
            (self.navigate_action_client, '/navigate_to_pose'),
            (self.charger_call_client, 'charger_call')
        ]
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service_name} service...')

    def battery_status_callback(self, msg):
        """Check battery status and take action."""
        battery_level = msg.data
        if battery_level < 20:
            self.get_logger().info('Battery level is low. Navigating to charger.')

            if self.charger_station:
                # Navigate to charging station
                if self.navigate_to_station(self.charger_station):
                    self.get_logger().info("Arrived at charging station. Starting charging process.")
                    self.start_charging()
                else:
                    self.get_logger().error("Failed to reach charging station.")
            else:
                self.get_logger().error('No charger station found in the configuration.')
        else:
            self.get_logger().info('Battery level is normal. No action needed.')

    def navigate_to_station(self, station):
        """Send a navigation goal to NavigateToPose."""
        if not self.navigate_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = station['x']
        goal_msg.pose.pose.position.y = station['y']
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation (yaw)
        goal_msg.pose.pose.orientation.z = station['yaw']
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.navigate_action_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            time.sleep(0.1)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected.")
            return False

        self.get_logger().info("Navigation goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            time.sleep(0.1)

        self.get_logger().info("Navigation completed successfully.")
        return True

    def start_charging(self):
        """Call the charger_call service to start charging."""
        request = SetBool.Request()
        request.data = True
        future = self.charger_call_client.call_async(request)
        while not future.done():
            time.sleep(0.1)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Charging started successfully.")
        else:
            self.get_logger().error("Failed to start charging.")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
