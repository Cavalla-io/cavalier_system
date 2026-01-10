import time
import rclpy
from rclpy.node import Node
from ros_system.functions.hardware_commands import check_drive_fault

#--------------------------------- HELPER FUNCTIONS ---------------------------------#

# Global variable to store last known drive status
last_drive_status = "unknown"

def check_drive_status(logger):
    global last_drive_status
    can = CanInterface()
    # Safely read message
    message_data = can.read_message(0x203)
    
    if message_data is None:
        logger.warning('Drive status check timed out (No CAN message 0x203), keeping last known status')
        return last_drive_status
        
    # message_data[0] returns the BitAccessibleByte
    # message_data[0][0] returns bit 0 of that byte
    drive_status_bit = message_data[0][0]
    
    if drive_status_bit == 1:
        status = "true"
    else:
        status = "false"
    
    last_drive_status = status
    return status


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
