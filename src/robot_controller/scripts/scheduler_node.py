#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from robot_interfaces.srv import Station2GO, Target2GO, SetID
from std_srvs.srv import SetBool

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')

        self.declare_parameter('station_path', 'robot_controller')
        self.station_path = self.get_parameter('station_path').value

        # โหลด Station List จากไฟล์ YAML
        path_file = os.path.join(get_package_share_directory(self.station_path), 'config', 'station.yaml')
        self.station_list = self.load_station(path_file)

        # สร้าง Service
        self.srv_station2go = self.create_service(Station2GO, 'station_to_go', self.station_to_go_callback)
        self.target_to_go_client = self.create_client(Target2GO, 'target_to_go')
        self.set_id_client = self.create_client(SetID, 'set_target_id')
        self.call_docking = self.create_client(SetBool, 'toggle_docking')

        self.get_logger().info('✅ Scheduler node has been started.')

    def load_station(self, path_file):
        """โหลดข้อมูลสถานีจากไฟล์ YAML"""
        if os.path.exists(path_file):
            try:
                with open(path_file, 'r') as file:
                    data = yaml.safe_load(file)
                    stations = data.get('path', data) if isinstance(data, dict) else data
                    self.get_logger().info(f"✅ Loaded {len(stations)} waypoints from {path_file}")
                    return stations
            except yaml.YAMLError as e:
                self.get_logger().error(f"❌ Error loading YAML file: {e}")
                return []
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            return []

    def station_to_go_callback(self, request, response):
        """Callback เมื่อได้รับคำขอไปยังสถานีที่กำหนด"""
        self.get_logger().info(f'📌 Received station request: {request.station}')

        station_data = next((s for s in self.station_list if s['name'] == request.station), None)

        if not station_data:
            response.success = False
            response.message = f"❌ Station '{request.station}' not found."
            return response

        self.get_logger().info(f"🚀 Navigating to station: {station_data}")

        # เรียก Target2GO Service
        if not self.call_target_to_go(station_data):
            response.success = False
            response.message = "❌ Failed to send target position."
            return response

        # เรียก SetID Service
        if not self.call_set_target_id(station_data['aruco_id']):
            response.success = False
            response.message = "❌ Failed to set target ID."
            return response
        
        # เรียก Toggle Docking Service
        if not self.call_toggle_docking(True):
            response.success = False
            response.message = "❌ Failed to toggle docking."
            return response

        response.success = True
        response.message = f"✅ Successfully sent robot to {request.station}"
        return response
    
    def call_toggle_docking(self, enable):
        """เรียก Service Toggle Docking"""
        if not self.call_docking.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("❌ Toggle Docking service not available!")
            return False

        docking_request = SetBool.Request()
        docking_request.data = enable

        future = self.call_docking.call_async(docking_request)
        future.add_done_callback(self.handle_toggle_docking_response)
        return True
    
    def handle_toggle_docking_response(self, future):
        """Callback เมื่อได้รับผลลัพธ์จาก Toggle Docking"""
        try:
            result = future.result()
            self.get_logger().info(f"✅ Toggle docking service succeeded: {result}")
        except Exception as e:
            self.get_logger().error(f"❌ Toggle docking service failed: {e}")

    
    def call_target_to_go(self, station_data):
        """เรียก Service Target2GO"""
        if not self.target_to_go_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("❌ Target2GO service not available!")
            return False

        target_request = Target2GO.Request()
        target_request.target.x = station_data['x']
        target_request.target.y = station_data['y']
        target_request.target.z = station_data['yaw']

        future = self.target_to_go_client.call_async(target_request)
        future.add_done_callback(self.handle_target_response)
        return True

    def handle_target_response(self, future):
        """Callback เมื่อได้รับผลลัพธ์จาก Target2GO"""
        try:
            result = future.result()
            self.get_logger().info(f"✅ Target to go service succeeded: {result}")
        except Exception as e:
            self.get_logger().error(f"❌ Target to go service failed: {e}")

    def call_set_target_id(self, aruco_id):
        """เรียก Service SetID"""
        if not self.set_id_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("❌ SetID service not available!")
            return False

        set_id_request = SetID.Request()
        set_id_request.aruco_id = aruco_id

        future = self.set_id_client.call_async(set_id_request)
        future.add_done_callback(self.handle_set_id_response)
        return True

    def handle_set_id_response(self, future):
        """Callback เมื่อได้รับผลลัพธ์จาก SetID"""
        try:
            result = future.result()
            self.get_logger().info(f"✅ Set target ID succeeded: {result}")
        except Exception as e:
            self.get_logger().error(f"❌ Set target ID failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
