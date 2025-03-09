#! /usr/bin/env python3

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node

from robot_navigator import BasicNavigator, NavigationResult
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from robot_interfaces.srv import Station2GO, Target2GO, SetID, MultiTarget2GO
from std_srvs.srv import SetBool

class InspectionNode(Node):
    def __init__(self):
        super().__init__('inspection_node')
        self.navigator = BasicNavigator()

        self.declare_parameter('station_path', 'robot_controller')
        self.station_path = self.get_parameter('station_path').value

        # โหลด Station List จากไฟล์ YAML
        path_file = os.path.join(get_package_share_directory(self.station_path), 'config', 'station.yaml')
        self.station_list = self.load_station(path_file)

        # สร้าง Service
        self.multi_target_service = self.create_service(MultiTarget2GO, 'multi_target_service', self.handle_multi_target_request)
        self.set_id_client = self.create_client(SetID, 'set_target_id')
        self.call_docking = self.create_client(SetBool, 'toggle_docking')
        
        # สถานะ Docking
        self.docking_status = False
        self.docking_service = self.create_service(SetBool, 'set_docking_status', self.set_docking_status_cb)

        self.get_logger().info('✅ Demo node has been started.')

    def set_docking_status_cb(self, request, response):
        """Callback สำหรับอัพเดทสถานะ docking"""
        self.docking_status = request.data
        response.success = True
        response.message = 'Docking status updated'
        return response

    def handle_multi_target_request(self, request, response):
        self.get_logger().info('🔔 Received request to send multiple targets.')
        
        self.get_logger().info(f'🚀 Moving to station {request.target_names}')
        response.success = True
        response.message = '🔔 Sending multiple'
        return response

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

    def run(self):
        self.navigator.waitUntilNav2Active()

        inspection_points = []
        for pt in self.station_list:
            inspection_pose = PoseStamped()
            inspection_pose.header.frame_id = 'map'
            inspection_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            inspection_pose.pose.position.x = pt['x']
            inspection_pose.pose.position.y = pt['y']
            inspection_pose.pose.orientation.z = pt['yaw']
            inspection_pose.pose.orientation.w = 1.0
            inspection_points.append(deepcopy(inspection_pose))

        for idx, (pt, target) in enumerate(zip(self.station_list, inspection_points)):
            self.get_logger().info(f'🚀 Moving to station {idx+1}/{len(inspection_points)}')
            
            # เดินทางไปยังจุดหมาย
            self.navigator.goToPose(target)
            while not self.navigator.isNavComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

            # เรียกใช้บริการสองครั้งตามลำดับ
            try:
                # เรียกใช้บริการแรก: ตั้งค่า ArUco ID
                aruco_id = pt['aruco_id']
                set_id_req = SetID.Request(aruco_id=aruco_id)
                if self.set_id_client.wait_for_service(timeout_sec=5.0):
                    future = self.set_id_client.call_async(set_id_req)
                    rclpy.spin_until_future_complete(self, future)
                    self.get_logger().info(f'🆔 Set ArUco ID to {aruco_id} success')
                else:
                    raise RuntimeError('Service /set_target_id not available')

                # เรียกใช้บริการที่สอง: เริ่มกระบวนการ Docking
                dock_req = SetBool.Request(data=True)
                if self.call_docking.wait_for_service(timeout_sec=5.0):
                    future = self.call_docking.call_async(dock_req)
                    rclpy.spin_until_future_complete(self, future)
                    self.get_logger().info('⚡ Activated docking sequence')
                else:
                    raise RuntimeError('Service /toggle_docking not available')

                # รอสถานะ Docking
                self.docking_status = False
                while not self.docking_status:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    time.sleep(0.1)
                self.get_logger().info('✅ Docking confirmed!')

                # Disable docking after completion
                try:
                    undock_req = SetBool.Request(data=False)
                    if self.call_docking.wait_for_service(timeout_sec=5.0):
                        future = self.call_docking.call_async(undock_req)
                        rclpy.spin_until_future_complete(self, future)
                        self.get_logger().info('⚓ Docking disabled')
                    else:
                        self.get_logger().error('❌ Service /toggle_docking not available when disabling')
                except Exception as e:
                    self.get_logger().error(f'❌ Error disabling docking: {e}')

            except Exception as e:
                self.get_logger().error(f'❌ Error at station {idx+1}: {str(e)}')
                continue

        self.get_logger().info('🎉 Mission completed!')

def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()