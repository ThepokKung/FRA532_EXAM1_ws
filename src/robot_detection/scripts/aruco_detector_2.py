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
        self.Kp_linear = 0.25
        self.Kp_angular = 0.5  # ลดค่าเพื่อลดการสั่น
        self.center_tolerance = 5  # พิกเซล (อนุญาตคลาดเคลื่อนได้ ±20 พิกเซล)
        
        # State variables
        self.last_horizontal_error = 0
        self.image_width = 0
        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)
        self.image_width = msg.width
        self.get_logger().info("Calibration received")

    def image_callback(self, msg):
        if self.camera_matrix is None or self.image_width == 0:
            return

        try:
            # แปลงภาพและตรวจจับ AruCo ก่อนวาดเส้นใดๆ
            raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            display_image = raw_image.copy()
            
            # ตรวจจับ AruCo
            corners, ids, _ = aruco.detectMarkers(raw_image, self.aruco_dict, parameters=self.parameters)
            
            cmd_vel = Twist()
            status = "SEARCHING"
            
            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                
                for i in range(len(ids)):
                    # คำนวณตำแหน่ง
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)
                    marker_center = np.mean(corners[i][0], axis=0)
                    horizontal_error = marker_center[0] - (self.image_width//2)
                    
                    # ควบคุมการเคลื่อนที่
                    if abs(horizontal_error) > self.center_tolerance:
                        # ปรับแนว
                        cmd_vel.angular.z = -self.Kp_angular * (horizontal_error / (self.image_width//2))
                        status = "ALIGNING"
                    elif distance > self.target_distance:
                        # เคลื่อนที่ตรง
                        cmd_vel.linear.x = self.Kp_linear * (distance - self.target_distance)
                        status = "APPROACHING"
                    else:
                        # หยุด
                        status = "DOCKED!"
                    
                    # วาดข้อมูลบนภาพ
                    cv2.drawFrameAxes(display_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                    cv2.putText(display_image, f"ID:{ids[i][0]} Dist:{distance:.2f}m", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                self.cmd_vel_pub.publish(Twist())
            
            # วาดเส้นกลางและสถานะ (หลังตรวจจับเสร็จ)
            cv2.line(display_image, 
                    (self.image_width//2, 0), 
                    (self.image_width//2, display_image.shape[0]), 
                    (0,255,255), 2)
            
            cv2.putText(display_image, status, 
                       (10,70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                       (0,255,0) if status=="DOCKED!" else (255,0,0), 2)
            
            cv2.imshow("AruCo Docking", display_image)
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