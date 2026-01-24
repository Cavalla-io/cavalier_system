import os
import subprocess
import json
import can
from ament_index_python.packages import get_package_share_directory

class BitAccessibleByte(int):
    # a wrapper around int that allows bit access.
    # Usage:
    #   byte = BitAccessibleByte(0x05)
    #   byte[0]  # Returns 1 (bit 0 is set)
    #   byte.bit(2) # Returns 1 (bit 2 is set)
    def __new__(cls, value):
        return super().__new__(cls, value)

    def __getitem__(self, bit_index):
        # allows access via byte[bit_index]
        if not (0 <= bit_index <= 7):
            raise IndexError("Bit index must be between 0 and 7")
        return (self >> bit_index) & 1

    def bit(self, bit_index):
        # readable method for bit access
        return self[bit_index]

class BitAccessibleData(bytes):
    # wrapper around bytes that returns BitAccessibleByte when indexed
    def __new__(cls, source):
        return super().__new__(cls, source)

    def __getitem__(self, index):
        val = super().__getitem__(index)
        return BitAccessibleByte(val)


class CanInterface:
    def __init__(self, interface="can0", bitrate=250000):
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None

    def check(self):
        # checks if the can interface is up using sysfs
        operstate_path = f"/sys/class/net/{self.interface}/operstate"
        if not os.path.exists(operstate_path):
            return False
            
        try:
            with open(operstate_path, 'r') as f:
                state = f.read().strip()
            # CAN interfaces often show as 'unknown' when up because they lack carrier detect
            # If it says 'down', it is definitely down.
            if state == 'down':
                return False
            return True
        except Exception:
            return False

    def setup(self):
        # sets up the can interface
        subprocess.run(["sudo", "ip", "link", "set", "down", self.interface])
        subprocess.run([
            "sudo", "ip", "link", "set", "up", 
            self.interface, 
            "type", "can", 
            "bitrate", str(self.bitrate)
        ])

    def connect(self):
        """Connects to the CAN bus using python-can."""
        try:
            self.bus = can.Bus(interface='socketcan', channel=self.interface, bitrate=self.bitrate)
            return True
        except can.CanError as e:
            print(f"Failed to connect to CAN bus: {e}")
            return False

    def send_message(self, arbitration_id, data, is_extended_id=False):
        # sends can message
        if not self.bus:
            if not self.connect():
                return False
        
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=is_extended_id)
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"Message not sent: {e}")
            return False

    def read_message(self, arbitration_id, timeout=1.0):
        # reads a single message from specified frame from the bus
        # returns data as a bytearray, or None if no message
        if not self.bus:
            if not self.connect():
                return None
        
        # apply filter to only see the requested ID (kernel mask)
        original_filters = self.bus.filters
        self.bus.set_filters([{"can_id": arbitration_id, "can_mask": 0x7FF, "extended": False}])

        try:
            msg = self.bus.recv(timeout=timeout)
            if msg:
                # Wrap the data in our custom class
                return BitAccessibleData(msg.data)
            return None
        except (can.CanError, OSError) as e:
            # Catch network down errors gracefully
            print(f"CAN Read Error: {e}")
            return None
        finally:
            # restore original filters (usually None/All)
            self.bus.set_filters(original_filters)

    def create_continuous_reader(self, arbitration_id):
        # returns a NEW can.Bus instance filtered for arbitration_id.
        # user must handle recv() loop on this bus object.
        # useful for high-frequency polling nodes.
        filters = [{"can_id": arbitration_id, "can_mask": 0x7FF, "extended": False}]
        try:
            return can.Bus(channel=self.interface, interface='socketcan', can_filters=filters)
        except Exception as e:
            print(f"Failed to create continuous reader: {e}")
            return None


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

def check_cameras():
    results = {}
    # cameras 20-26
    for i in range(20, 27):
        ip = f"192.168.2.{i}"
        # Ping with count 1 and timeout 1 second
        command = ["ping", "-c", "1", "-W", "1", ip]
        try:
            subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
            results[ip] = True
        except subprocess.CalledProcessError:
            results[ip] = False
    return results

def check_lidars():
    results = {}
    # lidars 75 80 85
    for i in [75, 80, 85]:
        ip = f"192.168.2.{i}"
        # Ping with count 1 and timeout 1 second
        command = ["ping", "-c", "1", "-W", "1", ip]
        try:
            subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
            results[ip] = True
        except subprocess.CalledProcessError:
            results[ip] = False
    return results

def setup_docker():
    print("todo: setup docker")
    pass

if __name__ == "__main__":
    can_interface = CanInterface()
    print(f"Checking {can_interface.interface} status:", can_interface.check())
    print("Checking cameras:", check_cameras())
    print("Checking lidars:", check_lidars())
