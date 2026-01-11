import os
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from ros_system.functions.setup_commands import CanInterface, check_ros, check_docker, setup_docker, check_cameras, check_lidars
from ros_system.functions.system_logging import create_log_file, create_log_ros_payload

#--------------------------------- HELPER FUNCTIONS ---------------------------------#

# Helper to load error codes efficiently
def load_error_codes():
    try:
        share_dir = get_package_share_directory('ros_system')
        json_path = os.path.join(share_dir, 'resource', 'cavalier_error_codes.json')
        with open(json_path, 'r') as f:
            return json.load(f)["errors"]
    except Exception as e:
        print(f"Failed to load error codes: {e}")
        return {}

# Load codes once globally
ERROR_CODES = load_error_codes()

def can_health(logger):
    can = CanInterface()
    can_result = can.check()
    if not can_result:
        logger.error('CAN is not running')
        try:
            can.setup()
            logger.info('CAN setup successful')
            # Warning that it had to restart
            error_payload = ERROR_CODES.get("SYS-001", {}).copy()
            error_payload["status"] = "RECOVERED"
            return error_payload
        except Exception as e:
            logger.error(f'Error setting up CAN: {e}')
            # Critical failure
            error_payload = ERROR_CODES.get("SYS-002", {}).copy()
            error_payload["details"] = str(e)
            return error_payload
    else:
        logger.debug('CAN is running')
        return True

def ros_health(logger):
    expected_topics = ['/joy', '/safety']
    ros_result = check_ros()
    # ros_result is tuple: (topic_process_result, node_process_result)
    
    for topic in expected_topics:
        if topic not in ros_result[0].stdout:
            logger.error(f'{topic} is not running')
            error_payload = ERROR_CODES.get("SYS-005", {}).copy()
            error_payload["details"] = f"Topic {topic} missing"
            return error_payload
    return True

def docker_health(logger):
    docker_result = check_docker()
    if docker_result.returncode != 0:
        logger.error('Docker is not running')
        try:
            setup_docker()
            logger.info('Docker setup successful')
            error_payload = ERROR_CODES.get("SYS-006", {}).copy()
            error_payload["status"] = "RECOVERED"
            return error_payload
        except Exception as e:
            logger.error(f'Error setting up Docker: {e}')
            error_payload = ERROR_CODES.get("SYS-006", {}).copy()
            error_payload["details"] = str(e)
            return error_payload
    else:
        logger.debug('Docker is running')
        return True



#--------------------------------- MAIN NODE ---------------------------------#

class SystemHealth(Node):
    def __init__(self):
        super().__init__('system_health')
        self.get_logger().info('System health node initialized')
        
        self.health_pub = self.create_publisher(String, '/cavalier_health', 10)
        
        # Use a timer instead of blocking while loop
        self.timer = self.create_timer(10.0, self.health_check)

    def health_check(self):
        self.get_logger().info('Performing health check...')
        can_payload = can_health(self.get_logger())
        ros_payload = ros_health(self.get_logger())
        docker_payload = docker_health(self.get_logger())
        cameras_payload = check_cameras()
        lidars_payload = check_lidars()

        # Collect active errors (anything that is NOT True)
        active_errors = [p for p in [can_payload, ros_payload, docker_payload] if p is not True]

        final_payload = {
            "timestamp": time.time(),
            "status": "OK" if not active_errors else "ERROR",
            "can_status": can_payload,
            "active_errors": active_errors,
            "cameras_status": cameras_payload,
            "lidars_status": lidars_payload
        }
        
        # Publish to /cavalier_health
        msg = String()
        msg.data = json.dumps(final_payload)
        self.health_pub.publish(msg)

        # Log to file if there are errors
        if active_errors:
            create_log_file("software", "health_monitor", final_payload)

def main(args=None):
    rclpy.init(args=args)
    system_health = SystemHealth()
    rclpy.spin(system_health)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
