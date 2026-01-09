from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forklift_control',
            namespace='controls',
            executable='ForkliftControlNode',
            name='teleop_control'
        )
    ])
        #TODO: add methods to start the system
        # first check system health over the health topic
        # fill in gaps in system_health (ex. if docker is not running, start it, if can is down, bring it back up)
        # fill in gaps in hardware_health (ex. if the drive is faulted, restart the forklift, if a sensor is down, try restarting it)
        # once the system is healthy, start the controller node and the driver aids
        # once all things have been started, stop the teleop start node

        # it is important to note that this will need to be able to be ran by the health 
        # monitor. The health monitor will also need to actually send something out so we
        # can display a sign on the front end showing if the controls are on or not

def main(args=None):
    rclpy.init(args=args)
    teleop_start = TeleopStart()
    rclpy.spin(teleop_start)
    teleop_start.destroy_node()
    rclpy.shutdown()

