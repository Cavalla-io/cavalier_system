import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import can

class Can:
    def __init__(self):
        # Initialize dedicated bus for this node with specific filters
        # This is much more efficient than opening/closing or changing filters constantly
        try:
            filters = [{"can_id": 0x383, "can_mask": 0x7FF, "extended": False}]
            self.bus = can.Bus(channel='can0', interface='socketcan', can_filters=filters)
            print("ForkliftState CAN bus connected")
        except Exception as e:
            print(f"ForkliftState CAN init failed: {e}")
            self.bus = None
    
    def get_steering_angle(self):
        if self.bus is None:
            return None

        # Non-blocking receive (timeout=0) because we are in a timer loop
        # We read until the buffer is empty to get the LATEST message
        latest_msg = None
        
        try:
            while True:
                msg = self.bus.recv(timeout=0)
                if msg is None:
                    break
                latest_msg = msg
        except (can.CanError, OSError):
            return None
        
        if latest_msg:
            # Bytes 0-1: Steering Angle (int16, Little Endian)
            # Use int.from_bytes for cleaner conversion
            if len(latest_msg.data) >= 2:
                raw_value = int.from_bytes(latest_msg.data[0:2], byteorder='little', signed=True)
                angle_degrees = raw_value * 0.01
                return angle_degrees
            
        return None

class ForkliftState(Node):
    def __init__(self):
        super().__init__('forklift_state')
        self.publisher_ = self.create_publisher(Float32, '/forklift/steering_angle', 10)
        
        # 0.01s (100Hz) to match wheel_tracker performance
        self.timer = self.create_timer(0.01, self.publish_state) 
        self.can_wrapper = Can()

    def publish_state(self):
        angle = self.can_wrapper.get_steering_angle()
        
        if angle is not None:
            msg = Float32()
            msg.data = angle
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForkliftState()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
