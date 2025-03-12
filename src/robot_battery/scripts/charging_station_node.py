#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from std_srvs.srv import SetBool

from robot_interfaces.srv import RobotStateUpdate, RobotStateCheck, RobotStationCheck, RobotStationUpdate, BatteryUpdate

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ChargingStationNode(Node):
    def __init__(self):
        super().__init__('ChargeStationNode')

        self.charger_call_server = self.create_service(SetBool, 'charger_call', self.charger_call_callback)

        # Battery level update
        self.battery_update_level_client_cbg = MutuallyExclusiveCallbackGroup()
        self.battery_update_level_client = self.create_client(
            BatteryUpdate, 'battery_level_update', callback_group=self.battery_update_level_client_cbg)

        # Robot state and station services
        self.robot_station_check_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_station_update_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_state_check_cbg = MutuallyExclusiveCallbackGroup()
        self.robot_state_update_cbg = MutuallyExclusiveCallbackGroup()

        self.robot_station_check = self.create_client(
            RobotStationCheck, '/check_robot_station', callback_group=self.robot_station_check_cbg)
        self.robot_station_update = self.create_client(
            RobotStationUpdate, '/update_robot_station', callback_group=self.robot_station_update_cbg)
        self.robot_state_check = self.create_client(
            RobotStateCheck, '/check_robot_state', callback_group=self.robot_state_check_cbg)
        self.robot_state_update = self.create_client(
            RobotStateUpdate, '/update_robot_state', callback_group=self.robot_state_update_cbg)

        self.get_logger().info('Charge station node has been started.')

    def charger_call_callback(self, request, response):
        """Callback function to handle charging requests."""
        if request.data:
            # Check if robot state allows navigation
            robot_state = self.check_robot_state()
            self.get_logger().info(f"Robot state received: {robot_state}")

            if robot_state and robot_state.strip() != "None":
                self.get_logger().error(f"Robot state is '{robot_state}', not 'None'. Charger denied.")
                response.success = False
                response.message = f"Charger denied. Robot state: '{robot_state}'."
                return response
            
            robot_station = self.check_robot_station()
            self.get_logger().info(f"Robot station received: {robot_station}")

            if robot_station and robot_station.strip() != "ChangeStation":
                self.get_logger().error(f"Robot station is '{robot_station}', not 'ChangeStation'. Charger denied.")
                response.success = False
                response.message = f"Charger denied. Robot station: '{robot_station}'."
                return response
            
            # Update battery level
            self.get_logger().info("Updating battery level to 100%...")
            if self.battery_level_update(100):
                self.get_logger().info("Charging station has been charged successfully.")
                response.success = True
                response.message = "Charging station has been charged."
            else:
                self.get_logger().error("Failed to update battery level.")
                response.success = False
                response.message = "Failed to update battery level."
            
            return response
        else:
            self.get_logger().info("Charging station has been stopped.")
            response.success = False
            response.message = "Charging station has been stopped."
            return response

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
        """Call the /check_robot_station service to get the current station's ArUco ID."""
        request = RobotStationCheck.Request()
        request.checkstation = True
        future = self.robot_station_check.call_async(request)  # ✅ Fixed this line
        
        while not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            self.get_logger().info(f"Robot station is at: {future.result().station}")
            return future.result().station
        else:
            self.get_logger().error("Failed to call /check_robot_station service.")
            return None
        
    def check_robot_state(self):
        """Call the /check_robot_state service to get the current robot state."""
        request = RobotStateCheck.Request()
        request.checkstate = True
        future = self.robot_state_check.call_async(request)

        while not future.done():  # Avoid blocking the service
            time.sleep(0.1)

        if future.result() is not None:
            self.get_logger().info(f"Robot is currently at state: {future.result().state}")
            return future.result().state
        else:
            self.get_logger().error("Failed to call /check_robot_state service.")
            return None
        
    def battery_level_update(self, level):
        """Update battery level via service."""
        request = BatteryUpdate.Request()
        request.battery_level = level
        future = self.battery_update_level_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is not None:
            self.get_logger().info("Battery level updated successfully.")
            return True
        else:
            self.get_logger().error("Battery level update failed.")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ChargingStationNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
