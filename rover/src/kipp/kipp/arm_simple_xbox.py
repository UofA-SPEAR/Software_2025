#!/usr/bin/env python3

# CONTROLLER BINDINGS
JOINT1_POS = 0      # D-pad UP
JOINT1_NEG = 1      # D-pad DOWN  
JOINT2_POS = 2      # D-pad LEFT
JOINT2_NEG = 3      # D-pad RIGHT
JOINT3_POS = 4      # Left bumper
JOINT3_NEG = 5      # Right bumper
JOINT4_AXIS = 0     # Left stick X
JOINT5_AXIS = 1     # Left stick Y  
JOINT6_AXIS = 3     # Right stick Y

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class SimpleArmControl(Node):
    def __init__(self):
        super().__init__('simple_arm_control')
        
        self.velocity_scale = 1.0
        
        self.joy_subscription = self.create_subscription(Joy, 'manual/joy2', self.joy_callback, 10)
        self.motor_publisher = self.create_publisher(Float64MultiArray, 'motor_commands', 10)
    
    def joy_callback(self, msg):
        if len(msg.axes) < 4 or len(msg.buttons) < 6:
            return
        
        joint_velocities = [0.0] * 6
        
        # Button controls for joints 1, 2, 3
        if msg.buttons[JOINT1_POS]:
            joint_velocities[0] = self.velocity_scale
        elif msg.buttons[JOINT1_NEG]:
            joint_velocities[0] = -self.velocity_scale
            
        if msg.buttons[JOINT2_POS]:
            joint_velocities[1] = self.velocity_scale
        elif msg.buttons[JOINT2_NEG]:
            joint_velocities[1] = -self.velocity_scale
            
        if msg.buttons[JOINT3_POS]:
            joint_velocities[2] = self.velocity_scale
        elif msg.buttons[JOINT3_NEG]:
            joint_velocities[2] = -self.velocity_scale
        
        # Axis controls for joints 4, 5, 6
        joint_velocities[3] = msg.axes[JOINT4_AXIS] * self.velocity_scale
        joint_velocities[4] = msg.axes[JOINT5_AXIS] * self.velocity_scale
        joint_velocities[5] = msg.axes[JOINT6_AXIS] * self.velocity_scale
        
        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_velocities
        self.motor_publisher.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimpleArmControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()