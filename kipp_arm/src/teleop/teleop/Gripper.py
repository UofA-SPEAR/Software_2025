import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
import time
import numpy as np
from rclpy.time import Time
import sys
import pygame
import can
from can import Message

class GripperControl(Node):

    def __init__(self):
        super().__init__('gripper_contorl')
        pygame.init()
        pygame.display.set_mode((100, 100))
        
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)

        self.node_id = 1 

        #Publsiher drive and steer commands
        self.gripper_cmds = self.create_publisher(msg_type = Float32MultiArray, topic = "gripper_topic", qos_profile = QoSProfile(depth=10))
                  
        timer_period = 0.05

        self.gripper_control = Float32MultiArray()
        self.gripper_control.data = [0.0]

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.gripper_control_callback)

        

    
    def gripper_control_callback(self):
        
        
        self.gripper_control.data[0] = 0.0 

        pygame.event.pump() 
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFTBRACKET]:
            
            self.gripper_control.data[0] = 1.0
            
        elif keys[pygame.K_RIGHTBRACKET]:
            
            self.gripper_control.data[0] = -1.0

        self.gripper_cmds.publish(self.gripper_control)

    
    
            
def main(args=None):
    rclpy.init(args=args)

   
    GripperNode = GripperControl()

    rclpy.spin(GripperNode)

    GripperNode.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()