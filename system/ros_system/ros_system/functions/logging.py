import os
import json
import datetime

def create_log_file(log_type, log_name, content):
    #decide where to save the log based on the log type
    if log_type == "software":
        log_path = os.path.expanduser("~/cavalier_system/system_logs/software")
    elif log_type == "hardware":
        log_path = os.path.expanduser("~/cavalier_system/system_logs/hardware")
    else:
        raise ValueError("Invalid log type")

    #get date and time for file name
    now = datetime.datetime.now()
    log_file_name = log_name + "_" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".json"

    #create the log file
    if not os.path.exists(log_path):
        os.makedirs(log_path)
    log_file_path = os.path.join(log_path, log_file_name)
    with open(log_file_path, 'w') as f:
        json.dump(content, f)

def create_log_ros_payload(log_type, log_name, content):
    if log_type == "software":
        topic = "/system_health"
    elif log_type == "hardware":
        topic = "/forklift_health"
    else:
        raise ValueError("Invalid log type")

    payload = {
        "log_type": log_type,
        "log_name": log_name,
        "content": content}

    return payload

if __name__ == "__main__":
    create_log_file("software", "test", "test")
