import time
import rclpy
from rclpy.node import Node
from ros_system.functions.setup_commands import check_can, check_ros, check_docker, check_drive_fault

class SystemHealth(Node):
    def __init__(self):
        super().__init__('system_health')
        self.get_logger().info('System health node initialized')
        while rclpy.ok():
            can_result = check_can()
            ros_result = check_ros()
            if "forklift_health" in ros_result.stdout:
                self.get_logger().info('Forklift health node is running')
            else:
                self.get_logger().error('Forklift health node is not running')
            docker_result = check_docker()
            drive_fault_result = check_drive_fault()
            time.sleep(10)
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    system_health = SystemHealth()
    rclpy.spin(system_health)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
