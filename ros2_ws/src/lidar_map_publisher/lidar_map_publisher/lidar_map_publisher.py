import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

class LidarMapPublisher(Node):
    def __init__(self):
        super().__init__('lidar_map_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_size = 100  # size of the map in squares
        self.map_resolution = 0.1  # size of each square in meters

    def listener_callback(self, msg):
        # Create an empty grid map
        grid = np.zeros((self.map_size, self.map_size), dtype=int)

        # Process LiDAR ranges
        for i, range in enumerate(msg.ranges):
            if range < msg.range_max:
                # Calculate angle of the scan
                angle = msg.angle_min + i * msg.angle_increment
                
                # Calculate x, y coordinates in the map
                x = int((range * np.cos(angle) + self.map_size * self.map_resolution / 2) / self.map_resolution)
                y = int((range * np.nan_to_num(np.sin(angle)) + self.map_size * self.map_resolution / 2) / self.map_resolution)

                # Mark the cell as free (0 is free, 100 is occupied)
                if 0 <= x < self.map_size and 0 <= y < self.map_size:
                    grid[x][y] = 0

        # Create and publish the OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_size
        map_info.info.height = self.map_size
        map_msg.info.origin.position.x = -self.map_size * self.map_resolution / 2
        map_msg.info.origin.position.y = -self.map_size * self.map_resolution / 2
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = grid.flatten().tolist()
        self.publisher.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
