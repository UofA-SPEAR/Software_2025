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


class Ros_2_Can(Node):

    def __init__(self):
        super().__init__('ros2can')


        self.drive_data = self.create_subscription(msg_type = Float32MultiArray, topic = "gripper_topic", qos_profile = rclpy.qos.qos_profile_system_default, callback= self.gripper_msg)
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        self.node_id = 1 

        timer_period = 0.01

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.ros2can)

    def gripper_msg(self, msg):
        
        self.grip_msg = msg
        self.grip_msg_timestamp = time.time()

    

    

    def ros2can(self):

        msg = self.gripper_2_can()
        self.can_publish(msg)

        pass


    def gripper_2_can(self):
        priority = 10
        frame_id = 0x10
        can_messages = []
        try:
            arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
            grip = self.grip_msg.data[0]
            
            grip_bytes = struct.pack('Bf', 40, grip)
            can_msg = can.Message(arbitration_id=arbitration_id, data=grip_bytes, is_extended_id=True)
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

        

def main(args=None):
    
    rclpy.init(args=args)

    Ros_2_Can_Node = Ros_2_Can()

    rclpy.spin(Ros_2_Can_Node)

    Ros_2_Can_Node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()