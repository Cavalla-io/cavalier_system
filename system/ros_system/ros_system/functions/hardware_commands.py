#TODO: this file will contain commands for the hardware of the forklift
# This will have commands to use the turnkey, check the controller status, 


import os
import subprocess
import json
from ament_index_python.packages import get_package_share_directory

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