#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import yaml
import time
import math
import threading
from ament_index_python.packages import get_package_share_directory

# Action and Service imports
from nav2_msgs.action import NavigateToPose
from robot_interfaces.srv import Station2GO, SetID, RobotStationCheck
from std_srvs.srv import SetBool
from rclpy.action import ActionClient


class StationNavigationAndDocking(Node):
    def __init__(self):
        super().__init__('station_navigation_and_docking')

        # Load station database
        self.station_path = 'robot_controller'  # Package containing station.yaml
        path_file = os.path.join(get_package_share_directory(self.station_path), 'config', 'station.yaml')
        self.station_list = self.load_station(path_file)

        # Create a service for single-station navigation
        self.station_service = self.create_service(
            Station2GO,
            '/station_2go',
            self.station_callback
        )

        # Create action client for Nav2 navigation
        self.navigate_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Service clients
        self.set_aruco_client = self.create_client(SetID, '/set_aruco_station_id')
        self.docking_client = self.create_client(SetBool, '/docking_with_aruco')
        self.docking_status_client = self.create_client(SetBool, '/docking_aruco_status')
        # self.check_staion_client = self.create_client(RobotStationCheck, '/robot_station_check')
        
        # Wait for services to be available
        self.wait_for_services()

        self.get_logger().info('StationNavigationAndDocking node initialized.')

    def load_station(self, filepath):
        """Load the station database from a YAML file."""
        try:
            with open(filepath, 'r') as f:
                stations = yaml.safe_load(f)
            self.get_logger().info(f"Loaded {len(stations)} stations from {filepath}")
            return stations
        except Exception as e:
            self.get_logger().error(f"Failed to load station file: {e}")
            return []

    def wait_for_services(self):
        """Wait for necessary services to be available."""
        services = [
            (self.set_aruco_client, '/set_aruco_station_id'),
            (self.docking_client, '/docking_with_aruco'),
            (self.docking_status_client, '/docking_aruco_status')
        ]
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service_name} service...')

    def station_callback(self, request, response):
        """Service callback for /station_2go."""
        target_station = request.station

        # Check if station exists
        station = next((st for st in self.station_list if st['name'] == target_station), None)
        if not station:
            self.get_logger().error(f"Station '{target_station}' not found in database.")
            response.success = False
            response.message = f"Station '{target_station}' not found."
            return response
        
        # Run navigation and docking in a separate thread
        threading.Thread(target=self.process_station, args=(station, response), daemon=True).start()
        return response

    def process_station(self, station, response):
        """Navigate to station and handle docking."""
        station_name = station['name']
        self.get_logger().info(f"Navigating to station: {station_name}")

        # Navigate to station
        if not self.navigate_to_station(station):
            self.get_logger().error(f"Navigation failed for station: {station_name}")
            response.success = False
            response.message = "Navigation failed."
            return

        # Set ArUco ID
        if not self.set_aruco_station_id(station['aruco_id']):
            self.get_logger().error(f"Failed to set ArUco ID for station: {station_name}")
            response.success = False
            response.message = "Failed to set ArUco ID."
            return

        # Initiate docking
        if not self.start_docking():
            self.get_logger().error(f"Docking initiation failed for station: {station_name}")
            response.success = False
            response.message = "Docking failed to start."
            return

        # Wait for docking completion
        if not self.wait_for_docking():
            self.get_logger().error(f"Docking did not complete at station: {station_name}")
            response.success = False
            response.message = "Docking failed."
            return

        self.get_logger().info(f"Successfully navigated and docked at station: {station_name}")
        response.success = True
        response.message = "Successfully reached and docked."

    def navigate_to_station(self, station):
        """Send a navigation goal to the Nav2 NavigateToPose action server."""
        if not self.navigate_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = station['x']
        goal_msg.pose.pose.position.y = station['y']
        goal_msg.pose.pose.position.z = 0.0

        yaw = station['yaw']
        goal_msg.pose.pose.orientation.z = yaw
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

        self.get_logger().info("Navigation completed.")
        return True

    def set_aruco_station_id(self, aruco_id):
        """Call the /set_aruco_station_id service to set the station's ArUco ID."""
        request = SetID.Request()
        request.aruco_id = aruco_id
        future = self.set_aruco_client.call_async(request)
        while not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            self.get_logger().info(f"ArUco ID {aruco_id} set successfully.")
            return True
        self.get_logger().error("Failed to set ArUco ID.")
        return False

    def start_docking(self):
        """Call /docking_with_aruco to initiate docking."""
        request = SetBool.Request()
        request.data = True
        future = self.docking_client.call_async(request)
        while not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            self.get_logger().info("Docking initiated successfully.")
            return True
        self.get_logger().error("Failed to start docking.")
        return False

    def wait_for_docking(self, timeout_sec=120):
        """Wait for /docking_aruco_status to confirm docking completion."""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            request = SetBool.Request()
            request.data = True
            future = self.docking_status_client.call_async(request)
            while not future.done():
                time.sleep(0.1)

            result = future.result()
            if result is not None and result.success:
                self.get_logger().info("Docking completed successfully.")
                return True
            time.sleep(1)

        self.get_logger().error("Docking did not complete in time.")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = StationNavigationAndDocking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
