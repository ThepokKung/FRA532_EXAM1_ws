#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import yaml
import math
import time
import threading
from ament_index_python.packages import get_package_share_directory

# Action and Service imports
from nav2_msgs.action import NavigateToPose
from robot_interfaces.srv import MultiTarget2GO, SetID
from std_srvs.srv import SetBool
from rclpy.action import ActionClient


class MultiStationNavigationAndDocking(Node):
    def __init__(self):
        super().__init__('multi_station_navigation_and_docking')
        
        # Use the "robot_controller" package for the station YAML file.
        self.station_path = 'robot_controller'
        path_file = os.path.join(get_package_share_directory(self.station_path), 'config', 'station.yaml')
        self.station_list = self.load_station(path_file)
        
        # Create the /multi_station_2go service.
        self.multi_station_service = self.create_service(
            MultiTarget2GO,
            '/multi_station_2go',
            self.multi_station_callback
        )
        
        # Create an action client for the Nav2 NavigateToPose action.
        self.navigate_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Create service clients for setting the ArUco station ID and starting docking.
        self.set_aruco_client = self.create_client(SetID, '/set_aruco_station_id')
        self.docking_client = self.create_client(SetBool, '/docking_with_aruco')
        
        # Create a service client for checking docking status.
        self.docking_status_client = self.create_client(SetBool, '/docking_aruco_status')
        
        # Wait for the service clients to become available.
        while not self.set_aruco_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_aruco_station_id service...')
        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /docking_with_aruco service...')
        while not self.docking_status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /docking_status service...')
        
        self.get_logger().info('MultiStationNavigationAndDocking node initialized.')
    
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
        
    def multi_station_callback(self, request, response):
        """
        Service callback for /multi_station_2go.
        Validates station names and spawns a background thread to run the navigation and docking sequence.
        """
        target_names = request.target_names
        
        # Validate that each target name exists in the loaded station list.
        for target in target_names:
            if not any(station['name'] == target for station in self.station_list):
                self.get_logger().error(f"Station name '{target}' not found in database.")
                response.success = False
                response.message = f"Station '{target}' not found in database."
                return response
        
        response.success = True  # All station names are valid.
        response.message = "All target stations are valid Task start."
        self.get_logger().info("All target station names are valid. Starting sequence in background...")
        
        # Process the navigation/docking sequence in a background thread.
        threading.Thread(target=self.process_stations, args=(target_names,), daemon=True).start()
        
        return response
    
    def process_stations(self, target_names):
        """Process navigation and docking for each station asynchronously."""
        for target in target_names:
            station = next(st for st in self.station_list if st['name'] == target)
            self.get_logger().info(f"Navigating to station: {target}")
            
            # Navigate to the station.
            if not self.navigate_to_station(station):
                self.get_logger().error(f"Navigation failed for station: {target}")
                continue
            
            # Upon successful navigation, set the ArUco ID.
            if not self.set_aruco_station_id(station['aruco_id']):
                self.get_logger().error(f"Failed to set ArUco ID for station: {target}")
                continue
            
            # Initiate docking.
            if not self.start_docking():
                self.get_logger().error(f"Docking initiation failed for station: {target}")
                continue
            
            # Wait for real docking completion via /docking_status.
            if not self.wait_for_docking():
                self.get_logger().error(f"Docking did not complete for station: {target}")
                continue
            
            self.get_logger().info(f"Successfully navigated and docked at station: {target}")
    
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
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = yaw
        goal_msg.pose.pose.orientation.w = 1.0
        
        send_goal_future = self.navigate_action_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            time.sleep(0.1)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected by the action server.")
            return False
        
        self.get_logger().info("Navigation goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            time.sleep(0.1)
        self.get_logger().info("Navigation action completed.")
        return True
    
    def set_aruco_station_id(self, aruco_id):
        """Call the /set_aruco_station_id service to set the current station's ArUco ID."""
        request = SetID.Request()
        request.aruco_id = aruco_id
        future = self.set_aruco_client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            self.get_logger().info(f"ArUco station ID {aruco_id} set successfully.")
            return True
        else:
            self.get_logger().error("Failed to call /set_aruco_station_id service.")
            return False
    
    def start_docking(self):
        """
        Call the /docking_with_aruco service to initiate docking.
        Assumes the docking process is handled externally.
        """
        request = SetBool.Request()
        request.data = True
        future = self.docking_client.call_async(request)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            self.get_logger().info("Docking initiated successfully.")
            return True
        else:
            self.get_logger().error("Failed to initiate docking.")
            return False
    
    def wait_for_docking(self, timeout_sec=120):
        """
        Poll the external /docking_status service until it returns success or until the timeout expires.
        """
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            request = SetBool.Request()
            # The request field can be used as needed; here we simply send True.
            request.data = True
            future = self.docking_status_client.call_async(request)
            while not future.done():
                time.sleep(0.1)
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info("Docking completed successfully.")
                return True
            time.sleep(0.5)
        self.get_logger().error("Docking did not complete within the timeout period.")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MultiStationNavigationAndDocking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
