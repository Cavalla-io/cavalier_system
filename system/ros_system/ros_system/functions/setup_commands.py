import os
import subprocess
import json
from ament_index_python.packages import get_package_share_directory

class CanInterface:
    def __init__(self, interface="can0", bitrate=250000):
        self.interface = interface
        self.bitrate = bitrate

    def check(self):
        """Checks if the CAN interface is UP."""
        can_check_command = ["sudo", "ip", "link", "show", self.interface]
        try:
            result = subprocess.run(can_check_command, capture_output=True, text=True, check=True)
            if "UP" in result.stdout:
                return True
            else:
                return False
        except subprocess.CalledProcessError:
            return False

    def setup(self):
        """Sets up the CAN interface."""
        subprocess.run([
            "sudo", "ip", "link", "set", "up", 
            self.interface, 
            "type", "can", 
            "bitrate", str(self.bitrate)
        ])

# Wrapper functions for backward compatibility with existing code
def check_can():
    can = CanInterface()
    return can.check()

def setup_can():
    can = CanInterface()
    can.setup()

def check_ros():
    check_topic_command = ('source /opt/ros/jazzy/setup.bash;ros2 topic list')
    check_node_command = ('source /opt/ros/jazzy/setup.bash;ros2 node list')
    topic_result = subprocess.run(check_topic_command, shell=True, capture_output=True, text=True, check=True)
    node_result = subprocess.run(check_node_command, shell=True, capture_output=True, text=True, check=True)
    return topic_result, node_result

def check_docker():
    check_docker_command = ["sudo","docker","ps","-a","--no-trunc"]
    result = subprocess.run(check_docker_command, capture_output=True, text=True, check=True)
    return result

def setup_docker():
    print("todo: setup docker")
    pass

if __name__ == "__main__":
    can = CanInterface()
    print(f"Checking {can.interface} status:", can.check())
