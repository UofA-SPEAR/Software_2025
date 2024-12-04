import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
import can
from can import Message
import time
import struct

drive_topic = "/Rover/drive_commands"
steer_topic = "/Rover/steer_commands"
joy_topic = "/Rover/Joy_Topic"

"""
Actuator ID:

Drive Motor Front Left =   10 
Drive MOtor Middle Left =  11
Drive Motor Back Left =    12
Drive Motor Front Right =  13
Drive Motor Middle Right = 14
Drive Motor Back Right =   15

Steering Motor Front Left =  20
Steering Motor Front Right = 21
Steering Motor Back Left =   22
Steering Motor Back Right =  23

"""

class Ros_2_Can(Node):

    def __init__(self):
        super().__init__('ros2can')

        self.start_time = time.time()
        self.drive_msg_timestamp = time.time()
        self.steer_msg_timestamp = time.time()
        self.drive_msg = Float32MultiArray()
        self.steer_msg = Float32MultiArray()

        self.drive_msg.data = [0.0]*6
        self.steer_msg.data = [0.0]*4

        self.home_last_toggle_time = time.time()
        self.toggle_debounce_time = 0.5

        self.drive_data = self.create_subscription(msg_type = Float32MultiArray, topic = drive_topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.drivemsg)

        self.steer_data = self.create_subscription(msg_type = Float32MultiArray, topic = steer_topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.steermsg)

        self.joy_data = self.create_subscription(msg_type = Joy, topic = joy_topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.joymsg)

        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)

        self.node_id = 1 

        timer_period = 0.01

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.ros2can)

    def drivemsg(self, msg):
        
        self.drive_msg = msg
        self.drive_msg_timestamp = time.time()

    def steermsg(self, msg):

        
        self.steer_msg = msg
        self.steer_msg_timestamp = time.time()

    def joymsg(self, msg):

        self.joy_msg = msg
        self.joy_msg_timestamp = time.time()

    def ros2can(self):

        timeout = 0.5
        
        # Convert ROS messages to CAN messages
        self.heartbeat()
        self.home_controls()

        drive_can_messages = self.drive_float32_can()
        steer_can_messages = self.steer_float32_can()

        if time.time() - self.drive_msg_timestamp > timeout:
            self.drive_msg.data = [0.0]*6
            self.get_logger().warn("Default Value being published, new data not recived")
        if time.time() - self.steer_msg_timestamp > timeout:
            self.steer_msg.data = [0.0]*4
            self.get_logger().warn("Default Value being published, new data not recived")
        # Publish CAN messages on the CAN bus
        self.can_publish(drive_can_messages)
        self.can_publish(steer_can_messages)


    def drive_float32_can(self):

        priority = 4  
        frame_id = 0x10   
            
        can_messages = []
        try:    
            for i in range(6):
                arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
                velocity = self.drive_msg.data[i]
                
                # Convert the 32-bit float velocity to 4 bytes
                velocity_bytes = struct.pack('Bf', i + 10, velocity)
                    
                # Create a CAN message
                can_msg = can.Message(arbitration_id=arbitration_id, data=velocity_bytes, is_extended_id=True)
                can_messages.append(can_msg)
            return can_messages
        
        except IndexError:
            self.get_logger().warn("CRITICAL: Steer message data array doesn't have enough elements.")
            
        except Exception as e:
            self.get_logger().warn(f"CRITICAL: Unexpected error in steer_float32_can: {e}")


    def steer_float32_can(self):

        priority = 5     
        frame_id = 0x11   
            
        can_messages = []

        try: 
            for acc_id in range(4):
                arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
                velocity = self.steer_msg.data[acc_id]
                
                # Convert the 32-bit float velocity to 4 bytes
                position_bytes = struct.pack('Bf', (acc_id + 20) ,velocity) 
                    
                # Create a CAN message
                can_msg = can.Message(arbitration_id=arbitration_id, data=position_bytes, is_extended_id=True)
                can_messages.append(can_msg)

            return can_messages
        
        
        except IndexError:
            self.get_logger().warn("CRITICAL: Steer message data array doesn't have enough elements.")
        
        except Exception as e:
            self.get_logger().warn(f"CRITICAL: Unexpected error in steer_float32_can: {e}")

    def can_publish(self, can_messages):

        
        try:
            for msg in can_messages:
                self.bus.send(msg)

        except TypeError:
            self.get_logger().error("CRTICAL: Message not being publised, type error")

        except Exception as e:
            self.get_logger().error(f"CRITICAL: Unexpected error in steer_float32_can: {e}")


    def home_controls(self):
        
        priority = 3    
        frame_id = 0x02
        current_time = time.time()
        try:
            if (current_time - self.home_last_toggle_time) > self.toggle_debounce_time and self.joy_msg.buttons[8]:
                arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
                can_msg = can.Message(arbitration_id=arbitration_id, is_extended_id=True)
                self.can_publish([can_msg])
                self.home_last_toggle_time = current_time

        except IndexError:
            self.get_logger().error("Index Error home_controls")

    def heartbeat(self):

        priority = 2    
        frame_id = 0x30
        arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
        current_time = int((time.time() - self.start_time)*100)
        print(current_time)
        current_time_bytes = struct.pack('i', current_time) 
        can_msg = can.Message(arbitration_id=arbitration_id, data=current_time_bytes, is_extended_id=True)
        self.can_publish([can_msg])
        

def main(args=None):
    
    rclpy.init(args=args)

    Ros_2_Can_Node = Ros_2_Can()

    rclpy.spin(Ros_2_Can_Node)

    Ros_2_Can_Node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()