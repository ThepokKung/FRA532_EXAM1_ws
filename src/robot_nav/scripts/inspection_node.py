#! /usr/bin/env python3

import time
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from robot_navigator import BasicNavigator
from robot_interfaces.srv import MultiTarget2GO, SetID
from std_srvs.srv import SetBool


class InspectionNode(Node):
    def __init__(self):
        super().__init__('inspection_node')
        self.navigator = BasicNavigator()

        self.declare_parameter('station_path', 'robot_controller')
        self.station_path = self.get_parameter('station_path').value

        # โหลด Station List จากไฟล์ YAML
        path_file = os.path.join(get_package_share_directory(
            self.station_path), 'config', 'station.yaml')
        self.station_list = self.load_station(path_file)

        # สร้าง Service
        self.multi_target_service_cb = MutuallyExclusiveCallbackGroup()
        self.multi_target_service = self.create_service(MultiTarget2GO, 'multi_target_service', self.handle_multi_target_request,callback_group=self.multi_target_service_cb)
        self.set_target_id_cb = MutuallyExclusiveCallbackGroup()
        self.set_id_client = self.create_client(SetID, 'set_target_id',callback_group=self.set_target_id_cb)
        self.call_docking_cb = MutuallyExclusiveCallbackGroup()
        self.call_docking = self.create_client(SetBool, 'toggle_docking',callback_group=self.call_docking_cb)

        # สถานะ Docking
        self.docking_status_cb = MutuallyExclusiveCallbackGroup()
        self.docking_status = False
        self.docking_service = self.create_service(SetBool, 'set_docking_status', self.set_docking_status_cb,callback_group=self.docking_status_cb)

        # Task Control
        self.current_task = None  # เก็บ Task ที่กำลังทำงานอยู่
        self.get_logger().info('✅ Inspection node started and waiting for tasks.')

    def set_docking_status_cb(self, request, response):
        """Callback สำหรับอัพเดทสถานะ docking"""
        self.docking_status = request.data
        response.success = True
        response.message = 'Docking status updated'
        return response

    def handle_multi_target_request(self, request, response):
        """Callback เมื่อได้รับคำขอ multi target"""
        self.get_logger().info(f'🔔 Received new multi-target request: {request.target_names}')

        # ✅ รีเซ็ต Task หากไม่มีงานที่กำลังทำอยู่
        if self.current_task and not self.current_task.done():
            self.get_logger().warn('⚠️ Previous mission still running. Skipping new request.')
            response.success = False
            response.message = 'Previous mission still running'
            return response

        # ✅ สร้าง Task ใหม่
        self.current_task = self.executor.create_task(self.run(request.target_names))
        self.get_logger().info(f'🚀 Started new mission: {request.target_names}')

        response.success = True
        response.message = 'Mission started'
        return response


    def load_station(self, path_file):
        """โหลดข้อมูลสถานีจากไฟล์ YAML"""
        if os.path.exists(path_file):
            try:
                with open(path_file, 'r') as file:
                    data = yaml.safe_load(file)
                    stations = data.get('path', data) if isinstance(
                        data, dict) else data
                    self.get_logger().info(
                        f"✅ Loaded {len(stations)} waypoints from {path_file}")
                    return stations
            except yaml.YAMLError as e:
                self.get_logger().error(f"❌ Error loading YAML file: {e}")
                return []
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            return []

    def run(self, target_names):
        """ฟังก์ชันหลักที่ทำการเคลื่อนที่ไปยังเป้าหมาย"""
        self.navigator.waitUntilNav2Active()

        # ✅ เรียงลำดับ Target ตามลำดับที่ถูกส่งมา
        target_stations = sorted(
            [pt for pt in self.station_list if pt['name'] in target_names],
            key=lambda x: target_names.index(x['name'])
        )

        if not target_stations:
            self.get_logger().warn('⚠️ No valid targets found. Skipping mission.')
            return
        
        self.get_logger().info(f'🚀 Starting mission with {len(target_stations)} waypoints.')

        for idx, pt in enumerate(target_stations):
            inspection_pose = PoseStamped()
            inspection_pose.header.frame_id = 'map'
            inspection_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            inspection_pose.pose.position.x = pt['x']
            inspection_pose.pose.position.y = pt['y']
            inspection_pose.pose.orientation.z = pt['yaw']
            inspection_pose.pose.orientation.w = 1.0

            self.get_logger().info(f'🚀 Moving to station {pt["name"]} ({idx+1}/{len(target_stations)})')

            # ✅ ส่งเป้าหมายให้ Nav2
            self.navigator.goToPose(inspection_pose)

            while not self.navigator.isNavComplete():
                self.get_logger().debug('⌛ Waiting for navigation to complete...')
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

            # ✅ ตรวจสอบผลลัพธ์ของ Nav2
            nav_result = self.navigator.getResult()
            self.get_logger().info(f'🛠️ Navigation result: {nav_result}')  

            if nav_result != nav_result.SUCCEEDED:
                self.get_logger().warn(f'❌ Navigation to {pt["name"]} failed. Skipping.')
                continue

            self.get_logger().info(f'✅ Successfully arrived at {pt["name"]}. Proceeding with tasks.')

            # ✅ ตรวจสอบ Service
            try:
                # ตั้งค่า ArUco ID
                aruco_id = pt['aruco_id']
                set_id_req = SetID.Request()
                set_id_req.aruco_id = aruco_id

                if self.set_id_client.wait_for_service(timeout_sec=5.0):
                    future = self.set_id_client.call_async(set_id_req)
                    while not future.done():
                        rclpy.spin_once(self, timeout_sec=0.1)

                    response = future.result()
                    self.get_logger().info(f'🆔 Set ArUco ID to {aruco_id} success: {response}')
                else:
                    raise RuntimeError('Service /set_target_id not available')

                # เปิด Docking
                dock_req = SetBool.Request()
                dock_req.data = True

                if self.call_docking.wait_for_service(timeout_sec=5.0):
                    future = self.call_docking.call_async(dock_req)
                    while not future.done():
                        rclpy.spin_once(self, timeout_sec=0.1)

                    response = future.result()
                    self.get_logger().info(f'⚡ Activated docking sequence: {response}')
                else:
                    raise RuntimeError('Service /toggle_docking not available')

                # ✅ รอสถานะ Docking
                self.docking_status = False
                while not self.docking_status:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    time.sleep(0.1)
                self.get_logger().info('✅ Docking confirmed!')

                # ปิด Docking หลังเสร็จสิ้น
                undock_req = SetBool.Request()
                undock_req.data = False

                if self.call_docking.wait_for_service(timeout_sec=5.0):
                    future = self.call_docking.call_async(undock_req)
                    while not future.done():
                        rclpy.spin_once(self, timeout_sec=0.1)

                    response = future.result()
                    self.get_logger().info(f'⚓ Docking disabled: {response}')
                else:
                    self.get_logger().error('❌ Service /toggle_docking not available when disabling')

            except Exception as e:
                self.get_logger().error(f'❌ Error at station {pt["name"]}: {str(e)}')
                continue

        self.get_logger().info('🎉 Mission completed!')
        self.current_task = None  # ✅ รีเซ็ต Task



def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()

    # ใช้ MultiThreadedExecutor เพื่อรองรับ task หลายตัวพร้อมกัน
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
