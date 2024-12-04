import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
import can
from can import Message
import time
import struct

#Topic names

"""
Actuator ID:
Shoulder Roll =
Shoulder Pitch =
Elbow Roll =
Elbow Pitch =
Wrist Roll =
Wrist Pitch  =
"""

joint_traj_topic = "/Arm_Group_controller/joint_trajectory"
joint_states_topics = "/joint_states_arm"

class Arm_2_Can(Node):
    def __init__(self):
        super().__init__('arm2can')

        self.node_id = 1

        self.joint_state = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.joint_state_msg = JointState()

        self.joint_state_msg.name = ["Shoulder Roll", "Shoulder Pitch", "Elbow Roll", "Elbow Pitch", "Wrist Roll", "Wrist Pitch"]
        self.joint_state_msg.position = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_vel = [0.0,0.0,0.0,0.0,0.0,0.0]



        self.joint_traj_sub = self.create_subscription(msg_type = JointTrajectory, topic = joint_traj_topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.joint_traj_msg)
        #self.joint_state_pub = self.create_publisher(msg_type = JointState, topic = joint_states_topics, qos_profile = QoSProfile(depth=10))
        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate=500000)

        timer_period = 0.1

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.arm2can)

    
    def joint_traj_msg(self, msg):
        joint_names  = ["Shoulder Roll", "Shoulder Pitch", "Elbow Roll", "Elbow Pitch", "Wrist Roll", "Wrist Pitch"]
        self.joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_vel = [0.0,0.0,0.0,0.0,0.0,0.0]
        
        for index in range(len(joint_names)):
            self.joint_pos[index] = self.joint_position_map(msg, joint_names[index])
            self.joint_vel[index] = self.joint_velocity_map(msg, joint_names[index])

    
    def arm2can(self):
        
        try:
            joint_pos_can = self.pos_2_float32(self.joint_pos)
        
        except:
            self.get_logger().warn("Not able to get Position data")
        #joint_vel_can = self.vel_2_float32(self.joint_vel)


        try:
            self.can_publish(joint_pos_can)

        except:
            self.get_logger().warn("Not able to Publish Message")
        #self.can_publish(joint_vel_can)
        

        #recv_msg = self.can_recive()
        #self.get_logger().warn("Rec")
        
        #self.float32_2_pos(recv_msg)
        #self.get_logger().warn("Rec COnv")
        
        #self.Joint_State_Update()
        #self.get_logger().warn("Joint Updated")


    def pos_2_float32(self, joint_pos):
        
        priority = 13 
        frame_id = 0x11   
            
        can_messages = []
          
        for i in range(len(joint_pos)):
            arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
            position = joint_pos[i]
            
            position_bytes = struct.pack('Bf', i+30, position)

            can_msg = can.Message(arbitration_id=arbitration_id, data=position_bytes, is_extended_id=True)
            can_messages.append(can_msg)

        return can_messages
    

    def vel_2_float32(self, joint_vel):

        priority = 14 
        frame_id = 0x10   
            
        can_messages = []
          
        for i in range(len(joint_vel)):
            arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
            velocity = joint_vel[i]
            velocity_bytes = struct.pack('Bf', i+40, velocity)

            can_msg = can.Message(arbitration_id=arbitration_id, data=velocity_bytes, is_extended_id=True)
            can_messages.append(can_msg)

        return can_messages



    def can_publish(self, can_messages):

        
        try:
            for msg in can_messages:
                self.bus.send(msg)

        except TypeError:
            self.get_logger().error("CRTICAL: Message not being publised, type error")

        except Exception as e:
            self.get_logger().error(f"CRITICAL: Unexpected error in steer_float32_can: {e}")

    
    def can_recive(self):

        self.get_logger().warn("Can Rec func ran")
        msg = self.bus.recv()
        self.get_logger().warn("msg recived")
        return msg


    
    def float32_2_pos(self, can_msg):

        arbitration_id = can_msg.arbitration_id
        frame_id = self.get_frame_id(arbitration_id)
    
        if frame_id == 0x21:

            can_data = can_msg.data 
            actuator_id, position = struct.unpack('Bf', can_data)
            if actuator_id in range(30,37):
                self.joint_state[actuator_id - 30] = position



    def get_frame_id(self, arbitration_id):
        return (arbitration_id >> 8) & 0xFFFF

    
    def joint_position_map(self, msg, joint_name):

        index = msg.joint_names.index(joint_name)
        pos_val = msg.points[0].positions[index]

        return pos_val
        

    def joint_velocity_map(self, msg, joint_name):
        
        index = msg.joint_names.index(joint_name)
        vel_val = msg.points[0].positions[index]
        
        return vel_val

    def Joint_State_Update(self):
        
        self.joint_state_msg.position = self.joint_state

        self.joint_state_pub.publish(self.joint_state_msg)


def main(args=None):
    
    rclpy.init(args=args)

    arm_can_Node = Arm_2_Can()

    rclpy.spin(arm_can_Node)

    arm_can_Node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
