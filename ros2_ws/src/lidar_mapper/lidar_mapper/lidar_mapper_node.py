import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

class LIDARMapNode(Node):
    def __init__(self):
        super().__init__('lidar_map_node')
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE  # Match publisher on /scan

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile)
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map_1',
            qos_profile)
        self.map_size = 150  # Define the size of the map (NxN)
        self.map_resolution = 0.1  # Define map resolution in meters (each cell is 5cm x 5cm)

    def lidar_callback(self, msg):
        """Process each incoming LIDAR scan message."""
        n = self.map_size
        grid = -np.ones((n, n), dtype=int)  # Initialize the grid to -1 (unknown)

        center_x, center_y = n // 2, n // 2  # Define the center of the grid

        for angle in range(len(msg.ranges)):
            distance = msg.ranges[angle]
            if np.isfinite(distance):
                # Calculate the endpoint of the beam (obstacle)
                obstacle_x = int((distance * np.cos(np.radians(angle)) + n * self.map_resolution / 2) / self.map_resolution)
                obstacle_y = int((distance * np.sin(np.radians(angle)) + n * self.map_resolution / 2) / self.map_resolution)
                line_points = self.bresenham_line(center_x, center_y, obstacle_x, obstacle_y)
                for (x, y) in line_points[:-1]:  # Exclude the last point (obstacle)
                    if 0 <= x < n and 0 <= y < n:
                        grid[y][x] = 0  # Mark as free
                if 0 <= obstacle_x < n and 0 <= obstacle_y < n:
                    grid[obstacle_y][obstacle_x] = 100  # Mark the obstacle as occupied
            else:
                # If the distance is inf, mark all cells along the beam path to the edge of the map as free
                max_distance = (self.map_size * self.map_resolution) / 2  # Arbitrary large distance
                far_x = int((max_distance * np.cos(np.radians(angle)) + n * self.map_resolution / 2) / self.map_resolution)
                far_y = int((max_distance * np.sin(np.radians(angle)) + n * self.map_resolution / 2) / self.map_resolution)
                line_points = self.bresenham_line(center_x, center_y, far_x, far_y)
                for (x, y) in line_points:
                    if 0 <= x < n and 0 <= y < n:
                        grid[y][x] = 0  # Mark as free

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "imu_link"
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = n
        map_msg.info.height = n
        map_msg.info.origin.position.x = -n * self.map_resolution / 2
        map_msg.info.origin.position.y = -n * self.map_resolution / 2
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = grid.flatten().tolist()
        self.publisher.publish(map_msg)
        self.get_logger().info('Published map grid on /map_1')

    def bresenham_line(self, x0, y0, x1, y1):
        """Generate integer coordinates on the line from (x0, y0) to (x1, y1) using Bresenham's algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x, y))  # Make sure the endpoint is included
        return points

def main(args=None):
    rclpy.init(args=args)
    lidar_map_node = LIDARMapNode()
    rclpy.spin(lidar_map_node)
    lidar_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
