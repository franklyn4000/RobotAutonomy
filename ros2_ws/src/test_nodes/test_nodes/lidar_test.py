import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.scan_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss

    def scan_callback(self, msg):
        print(msg.ranges[0])


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()

    rclpy.spin(tb3)  # execute tb3 node
    # blocks until the executor cannot work

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()