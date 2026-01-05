import rclpy
from rclpy.node import Node

class TeleopStart(Node):
    def __init__(self):
        super().__init__('teleop_start')
        self.get_logger().info('Teleop start node initialized')
        #TODO: add methods to start the system
        # first check system health over the health topic
        # fill in gaps in system_health (ex. if docker is not running, start it, if can is down, bring it back up)
        # fill in gaps in hardware_health (ex. if the drive is faulted, restart the forklift, if a sensor is down, try restarting it)
        # once the system is healthy, start the controller node and the driver aids
        # once all things have been started, stop the teleop start node

def main(args=None):
    rclpy.init(args=args)
    teleop_start = TeleopStart()
    rclpy.spin(teleop_start)
    teleop_start.destroy_node()
    rclpy.shutdown()