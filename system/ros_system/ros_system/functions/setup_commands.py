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

def check_drive_fault():
    check_fault_command = ["candump","can0,183:7ff","-n","1"]

    try:
        result = subprocess.run(check_fault_command, capture_output=True, text=True, check=True, timeout=1)
    except subprocess.TimeoutExpired:
        return "No fault"
    except Exception:
        return None
    result_stripped = result.stdout.strip()
    result_array = result_stripped.split()
    fault_byte = list(result_array[7])
    fault_code = fault_byte[0] + "-" + fault_byte[1]

    package_share_directory = get_package_share_directory('ros_system')
    json_path = os.path.join(package_share_directory, 'resource', 'curtis_f2a_fault_codes.json')
    
    with open(json_path, 'r') as f:
        json_code = json.load(f)

    fault_description = json_code["flash_code"][fault_code]

    return fault_description

if __name__ == "__main__":
    print("Checking CAN status:", check_can())
