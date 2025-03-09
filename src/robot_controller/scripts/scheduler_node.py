import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from robot_interfaces.srv import Station2GO, Target2GO, SetID


class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')

        self.declare_parameter('station_path', 'robot_controller')
        self.station_path = self.get_parameter('station_path').value

        self.station_list = []
        # --- Load Path ---
        path_file = os.path.join(get_package_share_directory(
            self.station_path), 'config', 'station.yaml')
        self.load_station(path_file)

        self.srv_station2go = self.create_service(
            Station2GO, 'station_to_go', self.station_to_go_callback)
        self.target_to_go_client = self.create_client(
            Target2GO, 'target_to_go')

        # Staion to go data
        self.station_to_go_aruco_id = 0
        self.station_to_go_x = 0.0
        self.station_to_go_y = 0.0
        self.station_to_go_yaw = 0.0

        self.get_logger().info('Scheduler node has been started.')

    def load_station(self, path_file):
        if os.path.exists(path_file):
            with open(path_file, 'r') as file:
                data = yaml.safe_load(file)
                # Support both dictionary with 'path' key and direct list.
                self.station_list = data.get(
                    'path', data) if isinstance(data, dict) else data
                self.get_logger().info(
                    f"✅ Loaded path with {len(self.station_list)} waypoints from {path_file}")
        else:
            self.get_logger().error(f"❌ Path file not found: {path_file}")
            self.path = []

    def station_to_go_callback(self, request, response):
        self.get_logger().info(f'Received station name: {request.station}')

        station_data = next(
            (station for station in self.station_list if station['name'] == request.station), None)
        if station_data:
            response.success = True
            response.message = f"Station {request.station} received successfully with data: {station_data}"
            self.station_to_go_aruco_id = station_data['aruco_id']
            self.station_to_go_x = station_data['x']
            self.station_to_go_y = station_data['y']
            self.station_to_go_yaw = station_data['yaw']
            self.get_logger().info(f"Station {request.station} data: {station_data}")

            while not self.target_to_go_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service target_to_go not available, waiting again...')

            target_request = Target2GO.Request()
            target_request.target.x = station_data['x']
            target_request.target.y = station_data['y']
            target_request.target.z = station_data['yaw']

            future = self.target_to_go_client.call_async(target_request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(
                    f"Target to go service call succeeded: {future.result()}")
            else:
                self.get_logger().error(
                    f"Target to go service call failed: {future.exception()}")

            # Send aruco_id to /set_target_id service
            set_id_client = self.create_client(SetID, 'set_target_id')
            while not set_id_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service set_target_id not available, waiting again...')

            set_id_request = SetID.Request()
            set_id_request.aruco_id = station_data['aruco_id']

            future = set_id_client.call_async(set_id_request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(
                    f"Set target ID service call succeeded: {future.result()}")
            else:
                self.get_logger().error(
                    f"Set target ID service call failed: {future.exception()}")
        else:
            response.success = False
            response.message = f"Station {request.station} not found in the path"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
