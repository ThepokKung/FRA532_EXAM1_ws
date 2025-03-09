#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from std_srvs.srv import SetBool
from robot_interfaces.srv import SetID

class ArucoDocker(Node):
    def __init__(self):
        super().__init__('aruco_docker')
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.camera_info_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service
        self.srv = self.create_service(SetBool, 'toggle_docking', self.toggle_docking_callback)
        self.srv_id = self.create_service(SetID, 'set_target_id', self.set_target_id_callback)
        
        # Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.15
        self.target_id = 4  # ID ที่ต้องการควบคุม
        self.target_distance = 0.5
        self.Kp_linear = 0.5
        self.Kp_angular = 0.2
        self.center_tolerance = 5
        
        # State variables
        self.docking_enabled = False  # เริ่มต้นปิดระบบ
        self.image_width = 0
        self.camera_matrix = None
        self.dist_coeffs = None

    def toggle_docking_callback(self, request, response):
        """เปิด-ปิดโหมด Docking"""
        self.docking_enabled = request.data
        response.success = True
        response.message = f"Docking {'enabled' if self.docking_enabled else 'disabled'}"
        return response
    
    def set_target_id_callback(self, request, response):
        """ตั้งค่า ID ที่ต้องการควบคุม"""
        self.target_id = request.aruco_id
        response.success = True
        response.message = f"Target ID set to {self.target_id}"
        return response

    def calculate_yaw(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        return np.degrees(np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]))
    
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)
        self.image_width = msg.width

    def image_callback(self, msg):
        if self.camera_matrix is None or self.image_width == 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
            
            cmd_vel = Twist()
            status = "SEARCHING ID : " + str(self.target_id)
            
            if ids is not None:
                # ประมวลผลทุก Marker ที่พบ
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 
                    self.marker_size, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                target_found = False
                for i in range(len(ids)):
                    current_id = ids[i][0]
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]
                    
                    # วาดข้อมูลทุก Marker
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    cv2.putText(cv_image, 
                               f"ID: {current_id}", 
                               tuple(map(int, corners[i][0][0])), 
                               cv2.FONT_HERSHEY_SIMPLEX, 
                               0.6, 
                               (0,255,0), 
                               2)
                    
                    # ตรวจสอบเฉพาะ Target ID เมื่อเปิดโหมด Docking
                    if self.docking_enabled and current_id == self.target_id:
                        target_found = True
                        distance = np.linalg.norm(tvec)
                        yaw = self.calculate_yaw(rvec)
                        marker_center = np.mean(corners[i][0], axis=0)
                        horizontal_error = marker_center[0] - (self.image_width//2)

                        # Distance
                        cv2.putText(cv_image, f"Target Distance: {distance:.2f}m", (10,35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                        # Yaw
                        cv2.putText(cv_image, f"Target Yaw: {yaw:.1f}", (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                        
                        # ควบคุมการเคลื่อนที่
                        if abs(horizontal_error) > self.center_tolerance:
                            cmd_vel.angular.z = -self.Kp_angular * (horizontal_error / (self.image_width//2))
                            status = "ALIGNING"
                        elif distance > self.target_distance:
                            cmd_vel.linear.x = self.Kp_linear * (distance - self.target_distance)
                            status = "APPROACHING"
                        else:
                            status = "DOCKED!"
                
                # ส่งคำสั่งเคลื่อนที่เฉพาะเมื่อพบ Target ID และเปิดโหมด
                if target_found:
                    self.cmd_vel_pub.publish(cmd_vel)
            
            # แสดงสถานะโหมด Docking
            mode_text = "DOCKING MODE: ON" if self.docking_enabled else "DOCKING MODE: OFF"
            text_size = cv2.getTextSize(mode_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            text_x = cv_image.shape[1] - text_size[0] - 10  # ขอบขวา
            text_y = cv_image.shape[0] - 10  # ขอบล่าง

            # Center line
            cv2.line(cv_image, (self.image_width//2,0), (self.image_width//2,cv_image.shape[0]), (0,255,255),2)
            # Status
            cv2.putText(cv_image, status, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if "DOCKED" in status else (255,0,0), 2)
            # Mode
            cv2.putText(cv_image, mode_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if self.docking_enabled else (0,0,255), 2)
            
            # แสดงภาพตลอดเวลา
            cv2.imshow("AruCo Docking", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDocker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()