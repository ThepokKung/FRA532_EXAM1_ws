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
        self.marker_size = 0.15  # ต้องตรงกับ Gazebo!
        self.target_distance = 0.5
        self.center_tolerance = 5    # พิกเซล
        self.angle_tolerance = 2.0   # องศา
        
        # PID Control
        self.Kp_linear = 0.3
        self.Ki_linear = 0.01
        self.Kd_linear = 0.05
        self.Kp_angular = 0.4
        self.Ki_angular = 0.01
        self.Kd_angular = 0.05
        
        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.image_width = 0
        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)
        self.image_width = msg.width

    def calculate_pid(self, error, dt):
        """คำนวณค่าสัญญาณ PID"""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp_angular*error + self.Ki_angular*self.integral + self.Kd_angular*derivative
        self.prev_error = error
        return output

    def image_callback(self, msg):
        if self.camera_matrix is None or self.image_width == 0:
            return

        try:
            # แปลงภาพและตรวจจับ AruCo
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
            
            cmd_vel = Twist()
            status = "SEARCHING"
            
            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 
                    self.marker_size, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # ใช้เวลาในการคำนวณ PID
                current_time = self.get_clock().now()
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                self.prev_time = current_time
                
                for i in range(len(ids)):
                    # คำนวณตำแหน่งและมุม
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)
                    marker_center = np.mean(corners[i][0], axis=0)
                    horizontal_error = marker_center[0] - (self.image_width//2)
                    
                    # คำนวณมุม Yaw
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                    yaw = np.degrees(np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]))
                    
                    # ควบคุมด้วย PID
                    if abs(horizontal_error) > self.center_tolerance:
                        # ปรับแนวตั้งด้วย PID
                        angular_output = self.calculate_pid(horizontal_error, dt)
                        cmd_vel.angular.z = -angular_output / 100  # ปรับ scale
                        status = "ALIGNING"
                    elif abs(yaw) > self.angle_tolerance:
                        # ปรับมุม Yaw
                        cmd_vel.angular.z = -self.Kp_angular * np.radians(yaw)
                        status = "ADJUSTING YAW"
                    elif distance > self.target_distance:
                        # เคลื่อนที่ตรง
                        cmd_vel.linear.x = self.Kp_linear * (distance - self.target_distance)
                        status = "APPROACHING"
                    else:
                        # หยุด
                        status = "DOCKED!"
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                    
                    # วาดข้อมูล
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i][0], tvec, 0.1)
                    cv2.putText(cv_image, 
                               f"X-Error: {horizontal_error:.1f}px | Yaw: {yaw:.1f}°", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    
                self.cmd_vel_pub.publish(cmd_vel)
            
            # วาดเส้นกลางและสถานะ
            cv2.line(cv_image, (self.image_width//2,0), (self.image_width//2,cv_image.shape[0]), (0,255,255),2)
            cv2.putText(cv_image, status, (10,70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0) if status=="DOCKED!" else (255,0,0), 2)
            
            cv2.imshow("AruCo Docking", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            self.cmd_vel_pub.publish(Twist())
            self.integral = 0.0  # รีเซ็ต integral เมื่อมี error

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDocker()
    node.prev_time = node.get_clock().now()  # ตั้งค่าเวลาเริ่มต้น
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()