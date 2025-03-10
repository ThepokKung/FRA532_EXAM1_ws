#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from std_srvs.srv import SetBool
from robot_interfaces.srv import SetID
import message_filters
from rclpy.qos import qos_profile_sensor_data
import sensor_msgs_py.point_cloud2 as pc2


class ArucoDocker(Node):
    def __init__(self):
        super().__init__('aruco_docker')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = message_filters.Subscriber(
            self, Image, '/depth_camera/image_raw', qos_profile=qos_profile_sensor_data
        )
        self.point_cloud_sub = message_filters.Subscriber(
            self, PointCloud2, '/depth_camera/points', qos_profile=qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/depth_camera/camera_info', self.camera_info_callback, 10
        )

        # Approximate time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.point_cloud_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_image_pc_callback)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Services
        self.srv = self.create_service(
            SetBool, 'toggle_docking', self.toggle_docking_callback)
        self.srv_id = self.create_service(
            SetID, 'set_target_id', self.set_target_id_callback)
        self.docking_starus = self.create_client(
            SetBool, '/set_docking_status')

        # Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.15
        self.target_id = 0
        self.target_distance = 0.5
        self.Kp_linear = 0.5
        self.Kp_angular = 0.2
        self.center_tolerance = 5

        # State variables
        self.docking_enabled = False
        self.image_width = 0
        self.camera_matrix = None
        self.dist_coeffs = None

        self.get_logger().info("Aruco Docker node has been started.")

    def toggle_docking_callback(self, request, response):
        self.docking_enabled = request.data
        response.success = True
        response.message = f"Docking {'enabled' if self.docking_enabled else 'disabled'}"
        return response

    def set_target_id_callback(self, request, response):
        self.target_id = request.aruco_id
        response.success = True
        response.message = f"Target ID set to {self.target_id}"
        return response

    def calculate_yaw(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        return np.degrees(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.image_width = msg.width

    def synced_image_pc_callback(self, image_msg, point_cloud_msg):
        if self.camera_matrix is None or self.image_width == 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            corners, ids, _ = aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.parameters)

            cmd_vel = Twist()
            status = "SEARCHING ID : " + str(self.target_id)
            target_found = False

            # Process point cloud
            points = pc2.read_points_numpy(
                point_cloud_msg, field_names=("x", "y", "z"), skip_nans=False)
            height = point_cloud_msg.height
            width = point_cloud_msg.width
            if height > 1 and width > 1:
                points_2d = points.reshape(height, width, 3)
            else:
                self.get_logger().warn("Unorganized point cloud, skipping")
                return

            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_size,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                for i in range(len(ids)):
                    current_id = ids[i][0]
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]
                    corner = corners[i][0]

                    # Draw axes and ID text
                    cv2.drawFrameAxes(cv_image, self.camera_matrix,
                                      self.dist_coeffs, rvec, tvec, 0.1)
                    cv2.putText(cv_image, f"ID: {current_id}",
                                tuple(map(int, corner[0])),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    if self.docking_enabled and current_id == self.target_id:
                        # Get marker center coordinates
                        marker_center = np.mean(corner, axis=0)
                        u = int(marker_center[0])
                        v = int(marker_center[1])

                        if 0 <= u < width and 0 <= v < height:
                            point = points_2d[v, u]
                            _, _, z_pc = point

                            if np.isnan(z_pc):
                                continue

                            distance = z_pc
                            yaw = self.calculate_yaw(rvec)
                            horizontal_error = marker_center[0] - \
                                (self.image_width // 2)

                            # Update display info
                            cv2.putText(cv_image, f"Target Distance: {distance:.4f}m",
                                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.putText(cv_image, f"Target Yaw: {yaw:.1f}",
                                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                            # Control logic
                            if abs(horizontal_error) > self.center_tolerance:
                                cmd_vel.angular.z = -self.Kp_angular * \
                                    (horizontal_error / (self.image_width//2))
                                status = "ALIGNING"
                            elif distance <= self.target_distance + 0.0002:
                                status = "DOCKED!"
                                # Replace the service call block under "DOCKED!" with:
                                if self.docking_starus.service_is_ready():
                                    request = SetBool.Request()
                                    request.data = True
                                    future = self.docking_starus.call_async(
                                        request)
                                    future.add_done_callback(
                                        self.handle_docking_status_response)
                                else:
                                    self.get_logger().error("Service /set_docking_status not available.")

                            elif distance > self.target_distance:
                                cmd_vel.linear.x = self.Kp_linear * \
                                    (distance - self.target_distance)
                                status = "APPROACHING"

                            target_found = True

                if target_found:
                    self.cmd_vel_pub.publish(cmd_vel)

            # Draw UI elements
            mode_text = "DOCKING MODE: ON" if self.docking_enabled else "DOCKING MODE: OFF"
            text_size = cv2.getTextSize(
                mode_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            text_x = cv_image.shape[1] - text_size[0] - 10
            text_y = cv_image.shape[0] - 10

            cv2.line(cv_image, (self.image_width//2, 0),
                     (self.image_width//2, cv_image.shape[0]), (0, 255, 255), 2)
            cv2.putText(cv_image, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if "DOCKED" in status else (255, 0, 0), 2)
            cv2.putText(cv_image, mode_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if self.docking_enabled else (0, 0, 255), 2)

            cv2.imshow("AruCo Docking", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            self.cmd_vel_pub.publish(Twist())

            # Add this method to the class:
    def handle_docking_status_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Docking status set successfully.")
            else:
                self.get_logger().warn("Failed to set docking status.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDocker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
