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
        self.center_tolerance = 15      # พิกเซล
        self.angle_tolerance = 5.0      # องศา
        
        # PID Control
        self.Kp_linear = 0.3
        self.Kp_angular = 0.2
        
        # State variables
        self.image_width = 0
        self.camera_matrix = None
        self.dist_coeffs = None

    def rotation_matrix_to_yaw(self, rotation_matrix):
        """คำนวณมุม Yaw อย่างง่าย"""
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
            status = "SEARCHING"
            
            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 
                    self.marker_size, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                for i in range(len(ids)):
                    # คำนวณตำแหน่งและมุม
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    yaw = self.rotation_matrix_to_yaw(rotation_matrix)
                    distance = np.linalg.norm(tvec)
                    
                    # คำนวณตำแหน่งแนวตั้ง
                    marker_center = np.mean(corners[i][0], axis=0)
                    horizontal_error = marker_center[0] - (self.image_width//2)
                    
                    # เงื่อนไขควบคุมใหม่ (ปรับให้เคลื่อนที่ง่ายขึ้น)
                    if abs(yaw) > self.angle_tolerance:
                        # ปรับมุม Yaw
                        cmd_vel.angular.z = -self.Kp_angular * np.sign(yaw)
                        status = "ADJUSTING YAW"
                    elif abs(horizontal_error) > self.center_tolerance:
                        # ปรับตำแหน่งแนวตั้ง
                        cmd_vel.angular.z = -self.Kp_angular * (horizontal_error/(self.image_width//2))
                        status = "ALIGNING POSITION"
                    else:
                        # เคลื่อนที่ตรงเข้าหา
                        cmd_vel.linear.x = self.Kp_linear * (distance - self.target_distance)
                        status = "APPROACHING"
                        if distance <= self.target_distance:
                            status = "DOCKED!"
                            cmd_vel.linear.x = 0.0
                    
                    # วาดข้อมูล
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    cv2.putText(cv_image, 
                               f"Yaw: {yaw:.1f}° | Dist: {distance:.2f}m | X-Err: {horizontal_error:.1f}px", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    
                self.cmd_vel_pub.publish(cmd_vel)
            
            # วาดเส้นกลางและสถานะ
            cv2.line(cv_image, (self.image_width//2,0), (self.image_width//2,cv_image.shape[0]), (0,255,255),2)
            text_color = (0,255,0) if "DOCKED" in status else (255,0,0)
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