import time
import rclpy
from rclpy.node import Node
from ros_system.functions.hardware_commands import check_drive_fault

class ForkliftHealth(Node):
    def __init__(self):
        super().__init__('forklift_health')
        self.get_logger().info('Forklift health node initialized')

        drive_fault_result = check_drive_fault()
        self.get_logger().info(f'Drive fault: {drive_fault_result}')

def main(args=None):
    rclpy.init(args=args)
    forklift_health = ForkliftHealth()
    rclpy.spin(forklift_health)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
