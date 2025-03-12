#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import yaml
import time
import threading
from ament_index_python.packages import get_package_share_directory

# Action and Service imports
from nav2_msgs.action import NavigateToPose
from robot_interfaces.srv import Station2GO, SetID, RobotStationCheck, RobotStationUpdate, RobotStateCheck, RobotStateUpdate
from std_srvs.srv import SetBool
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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

        # Create service clients
        self.set_aruco_client_cbg = MutuallyExclusiveCallbackGroup()
        self.docking_client_cbg = MutuallyExclusiveCallbackGroup()
        self.docking_status_client_cbg = MutuallyExclusiveCallbackGroup()
        self.set_aruco_client = self.create_client(SetID, '/set_aruco_station_id', callback_group=self.set_aruco_client_cbg)
        self.docking_client = self.create_client(SetBool, '/docking_with_aruco',callback_group=self.docking_client_cbg)
        self.docking_status_client = self.create_client(SetBool, '/docking_aruco_status',callback_group=self.docking_status_client_cbg)

        # Robot state and station services
        self.robot_station_check_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_station_update_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_state_check_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_state_update_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_station_check = self.create_client(RobotStationCheck, '/check_robot_station', callback_group=self.robot_station_check_cbg)
        self.robot_station_update = self.create_client(RobotStationUpdate, '/update_robot_station',callback_group=self.robot_station_update_cbg)
        self.robot_state_check = self.create_client(RobotStateCheck, '/check_robot_state',callback_group=self.robot_state_check_cbg)
        self.robot_state_update = self.create_client(RobotStateUpdate, '/update_robot_state',callback_group=self.robot_state_update_cbg)

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
            (self.docking_status_client, '/docking_aruco_status'),
            (self.robot_station_check, '/check_robot_station'),
            (self.robot_station_update, '/update_robot_station'),
            (self.robot_state_check, '/check_robot_state'),
            (self.robot_state_update, '/update_robot_state'),
        ]
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service_name} service...')

    def station_callback(self, request, response):
        """Service callback for /station_2go."""
        target_station = request.station

        # Check if robot state allows navigation
        robot_state = self.check_robot_state()
        if robot_state and robot_state.strip() != "None":
            self.get_logger().error(f"Robot state is '{robot_state}', not 'None'. Navigation denied.")
            response.success = False
            response.message = f"Navigation denied. Robot state: '{robot_state}'."
            return response

        # Check if station exists
        station = next((st for st in self.station_list if st['name'] == target_station), None)
        if not station:
            self.get_logger().error(f"Station '{target_station}' not found in database.")
            response.success = False
            response.message = f"Station '{target_station}' not found."
            return response
        
        response.success = True
        response.message = "Task started."
        
        # Run navigation and docking in a separate thread
        threading.Thread(target=self.process_station, args=(station,), daemon=True).start()
        return response

    def process_station(self, station):
        """Navigate to station and handle docking."""
        station_name = station['name']
        self.get_logger().info(f"Navigating to station: {station_name}")

        # Update robot state
        if not self.update_robot_state('NAVIGATING'):
            self.get_logger().error("Failed to update robot state to 'NAVIGATING'.")

        # Navigate to station
        if not self.navigate_to_station(station):
            self.get_logger().error(f"Navigation failed for station: {station_name}")
            return

        # Set ArUco ID
        if not self.set_aruco_station_id(station['aruco_id']):
            self.get_logger().error(f"Failed to set ArUco ID for station: {station_name}")
            return

        # Initiate docking
        if not self.start_docking():
            self.get_logger().error(f"Docking initiation failed for station: {station_name}")
            return

        # Wait for docking completion
        if not self.wait_for_docking():
            self.get_logger().error(f"Docking did not complete at station: {station_name}")
            return

        # Update robot station
        if not self.update_robot_station(station_name):
            self.get_logger().error(f"Failed to update robot station to: {station_name}")

        # Reset robot state
        if not self.update_robot_state('None'):
            self.get_logger().error("Failed to update robot state to 'None'.")
        
        self.get_logger().info(f"Successfully completed navigation and docking at {station_name}.")

    def navigate_to_station(self, station):
        """Send a navigation goal to the Nav2 NavigateToPose action server."""
        if not self.navigate_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = station['x']
        goal_msg.pose.pose.position.y = station['y']
        goal_msg.pose.pose.orientation.z = station['yaw']
        goal_msg.pose.pose.orientation.w = 1.0  # Simplified orientation

        send_goal_future = self.navigate_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected.")
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        return True

    def set_aruco_station_id(self, aruco_id):
        """Call the /set_aruco_station_id service."""
        request = SetID.Request()
        request.aruco_id = aruco_id
        future = self.set_aruco_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def start_docking(self):
        """Call the /docking_with_aruco service."""
        request = SetBool.Request()
        request.data = True
        future = self.docking_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def wait_for_docking(self, timeout_sec=120):
        """Poll /docking_aruco_status service until docking completes."""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            request = SetBool.Request()
            request.data = True
            future = self.docking_status_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().success:
                return True
            time.sleep(0.5)
        return False

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


def main(args=None):
    rclpy.init(args=args)
    node = StationNavigationAndDocking()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
