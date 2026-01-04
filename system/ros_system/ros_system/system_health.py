import time
import rclpy
from rclpy.node import Node
from ros_system.functions.setup_commands import check_can, setup_can, check_ros, check_docker, check_drive_fault

def can_health(logger):
    can_result = check_can()
    if can_result == False:
        logger.error('CAN is not running')
        try:
            setup_can()
            logger.info('CAN setup successful')
        except Exception as e:
            logger.error(f'Error setting up CAN: {e}')
            return False
        time.sleep(1)
    else:
        logger.debug('CAN is running')
        time.sleep(1)

def ros_health(logger):
    expected_nodes = ['system_health',]
    expected_topics = ['/joy', '/safety']
    ros_result = check_ros()
    for topic in expected_topics:
        if topic not in ros_result:
            logger.error(f'{topic} is not running')
            return False
    return True

class SystemHealth(Node):
    def __init__(self):
        super().__init__('system_health')
        self.get_logger().info('System health node initialized')
        while rclpy.ok():
            can_health(self.get_logger())
            ros_health(self.get_logger())
            docker_result = check_docker()
            drive_fault_result = check_drive_fault()
            print(drive_fault_result)
            time.sleep(10)
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    system_health = SystemHealth()
    rclpy.spin(system_health)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
