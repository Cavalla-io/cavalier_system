import time
import rclpy
from rclpy.node import Node
from ros_system.functions.setup_commands import check_can, check_ros, check_docker, check_drive_fault

class ForkliftHealth(Node):
    def __init__(self):
        super().__init__('forklift_health')
        self.get_logger().info('Forklift health node initialized')


def main(args=None):
    rclpy.init(args=args)
    forklift_health = ForkliftHealth()
    rclpy.spin(forklift_health)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
