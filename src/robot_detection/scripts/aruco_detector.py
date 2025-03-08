#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.camera_info_callback, 10)
        
        # AruCo Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.15  # ต้องตรงกับขนาดใน Gazebo
        
        # Camera Parameters
        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Camera calibration received!")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
            
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect Markers
            corners, ids, _ = aruco.detectMarkers(
                cv_image, 
                self.aruco_dict,
                parameters=self.parameters
            )
            
            if ids is not None:
                # Estimate Pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners, 
                    self.marker_size, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # Draw Markers
                cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
                for i in range(len(ids)):
                    cv2.drawFrameAxes(
                        cv_image, 
                        self.camera_matrix, 
                        self.dist_coeffs, 
                        rvec[i], 
                        tvec[i], 
                        0.1  # แกนยาว 0.1 เมตร
                    )
                
                self.get_logger().info(f"Detected ID: {ids[0]} | Position: {tvec[0][0]}")

            # แสดงภาพด้วย OpenCV (หรือส่งไป Pygame)
            cv2.imshow("AruCo Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    detector = ArucoDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()