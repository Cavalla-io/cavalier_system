import os
import subprocess
import json
from ament_index_python.packages import get_package_share_directory

def check_can():
    can_check_command = ["sudo", "ip", "link", "show", "can0"]
    result = subprocess.run(can_check_command, capture_output=True, text=True, check=True)
    if "UP" in result.stdout:
        final_result = True
    else:
        final_result = False
    return final_result

def setup_can():
    subprocess.run(["sudo", "ip", "link", "set", "up", "can0", "type", "can", "bitrate", "250000"])
    pass

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
    print("Checking CAN status:", check_can())
