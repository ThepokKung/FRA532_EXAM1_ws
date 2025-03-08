#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDocker(Node):
    def __init__(self):
        super().__init__('aruco_docker')
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.camera_info_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.15
        self.target_distance = 0.5
        self.Kp_linear = 0.1
        self.Kp_angular_position = 0.15  # สำหรับปรับตำแหน่ง
        self.Kp_angular_yaw = 0.3       # สำหรับปรับมุม
        self.center_tolerance = 10       # พิกเซล
        self.yaw_tolerance = 0.1         # เรเดียน (~5.7 องศา)
        
        # State variables
        self.image_width = 0
        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)
        self.image_width = msg.width

    def rotation_matrix_to_yaw(self, rotation_matrix):
        """คำนวณมุม Yaw จาก Rotation Matrix"""
        return np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])

    def image_callback(self, msg):
        if self.camera_matrix is None or self.image_width == 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
            
            cmd_vel = Twist()
            status = "SEARCHING"
            
            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                
                for i in range(len(ids)):
                    # คำนวณตำแหน่งและมุม
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)
                    marker_center = np.mean(corners[i][0], axis=0)
                    horizontal_error = marker_center[0] - (self.image_width//2)
                    
                    # คำนวณมุม Yaw จาก Rotation Vector
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                    yaw = self.rotation_matrix_to_yaw(rotation_matrix)
                    
                    # ควบคุมการเคลื่อนที่ 2 ชั้น
                    if abs(horizontal_error) > self.center_tolerance:
                        # ปรับตำแหน่งแนวตั้ง
                        cmd_vel.angular.z = -self.Kp_angular_position * (horizontal_error / (self.image_width//2))
                        status = "ALIGNING POSITION"
                    elif abs(yaw) > self.yaw_tolerance:
                        # ปรับมุม Yaw
                        cmd_vel.angular.z = -self.Kp_angular_yaw * yaw
                        status = "ALIGNING YAW"
                    elif distance > self.target_distance:
                        # เคลื่อนที่ตรง
                        cmd_vel.linear.x = self.Kp_linear * (distance - self.target_distance)
                        status = "APPROACHING"
                    else:
                        # หยุด
                        status = "DOCKED!"
                    
                    # วาดข้อมูล
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                    cv2.putText(cv_image, 
                               f"Yaw: {np.degrees(yaw):.1f} deg | Dist: {distance:.2f}m", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    
                self.cmd_vel_pub.publish(cmd_vel)
            
            # วาดเส้นกลาง
            cv2.line(cv_image, 
                    (self.image_width//2, 0), 
                    (self.image_width//2, cv_image.shape[0]), 
                    (0,255,255), 2)
            
            # แสดงสถานะ
            text_color = (0,255,0) if status=="DOCKED!" else (255,0,0)
            cv2.putText(cv_image, status, (10,70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2)
            
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