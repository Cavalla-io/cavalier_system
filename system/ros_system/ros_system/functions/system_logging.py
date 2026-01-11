import os
import json
import datetime
import glob
import time

def create_log_file(log_type, log_name, content):
    #decide where to save the log based on the log type
    if log_type == "software":
        log_path = os.path.expanduser("~/cavalier_system/system_logs/software")
    elif log_type == "hardware":
        log_path = os.path.expanduser("~/cavalier_system/system_logs/hardware")
    else:
        raise ValueError("Invalid log type")

    if not os.path.exists(log_path):
        os.makedirs(log_path)

    # Wrap content with a timestamp
    new_entry = {
        "timestamp": datetime.datetime.now().isoformat(),
        "data": content
    }

    # Find the most recent log file for this log_name
    # Pattern: log_name_YYYY-MM-DD_HH-MM-SS.json
    search_pattern = os.path.join(log_path, f"{log_name}_*.json")
    files = glob.glob(search_pattern)
    
    current_file_path = None
    now = datetime.datetime.now()

    if files:
        # Sort by modification time, newest first
        files.sort(key=os.path.getmtime, reverse=True)
        latest_file = files[0]
        
        # Check if it was created/modified within the last hour
        # Note: Using creation time extraction from filename is safer than file metadata
        try:
            # Extract timestamp from filename: log_name_YYYY-MM-DD_HH-MM-SS.json
            timestamp_str = latest_file.split(f"{log_name}_")[-1].replace(".json", "")
            file_time = datetime.datetime.strptime(timestamp_str, "%Y-%m-%d_%H-%M-%S")
            
            time_diff = now - file_time
            if time_diff.total_seconds() < 3600:
                current_file_path = latest_file
        except ValueError:
            # If filename parsing fails, ignore and create new
            pass

    if current_file_path:
        # Append to existing file
        try:
            with open(current_file_path, 'r+') as f:
                try:
                    data = json.load(f)
                    if isinstance(data, list):
                        data.append(new_entry)
                    else:
                        # Convert single object to list if necessary
                        data = [data, new_entry]
                except json.JSONDecodeError:
                    # File is empty or corrupted, start fresh list
                    data = [new_entry]
                
                # Rewind and write
                f.seek(0)
                json.dump(data, f, indent=2)
                f.truncate()
        except Exception as e:
            print(f"Error appending to log file: {e}")
            # Fallback: create new file if append fails
            current_file_path = None

    # If no valid recent file found, create a new one
    if not current_file_path:
        log_file_name = log_name + "_" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".json"
        log_file_path = os.path.join(log_path, log_file_name)
        
        with open(log_file_path, 'w') as f:
            # Start as a list so we can append later
            json.dump([new_entry], f, indent=2)

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
    create_log_file("software", "test", "test message 1")
    time.sleep(2)
    create_log_file("software", "test", "test message 2 (should be in same file)")
