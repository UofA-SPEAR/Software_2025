#!/usr/bin/env python3

import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from numpy import linalg as LA

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

# CONTROLLER BINDINGS - Edit these to match your controller
# ========================================================

# BUTTONS
MODE_TOGGLE_BUTTON = 10      # BACK button
RESET_BUTTON = 11            # START button

# Joint Mode Controls
JOINT1_POS_BUTTON = 0      # D-pad UP
JOINT1_NEG_BUTTON = 1      # D-pad DOWN  
JOINT2_POS_BUTTON = 3      # D-pad LEFT
JOINT2_NEG_BUTTON = 4      # D-pad RIGHT
JOINT4_POS_BUTTON = 6       # Left shoulder
JOINT4_NEG_BUTTON = 7      # Right shoulder

# Cartesian Mode Controls
ROLL_LEFT_BUTTON = 6        # Left shoulder
ROLL_RIGHT_BUTTON = 7      # Right shoulder

# AXES
MOVE_X_AXIS = 0             # Left stick X
MOVE_Y_AXIS = 1             # Left stick Y
MOVE_Z_LEFT_TRIGGER = 5     # Left trigger
MOVE_Z_RIGHT_TRIGGER = 4    # Right trigger
PITCH_AXIS = 3              # Right stick Y
YAW_AXIS = 2                # Right stick X

# Joint Mode Axes
JOINT5_AXIS = 0             # Left stick X
JOINT6_AXIS = 1             # Left stick Y  
JOINT7_AXIS = 3             # Right stick Y

# ======================================================== 

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Control configuration
        self.joint_mode = False
        self.scale = 5.0
        self.rotation_scale = 25.0
        self.dt = 0.01
        self.deadzone = 0.15
        
        # Button state tracking
        self.prev_back_button = 0
        self.prev_start_button = 0
        
        # Robot definition
        self.links = [
            OriginLink(),
            URDFLink("base_rotation", [0, 0, 0.1], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi)),
            URDFLink("shoulder_lift", [0.05, 0, 0], [0, 0, np.pi], [1, 0, 0], bounds=(-np.pi/2, np.pi/2)),
            URDFLink("fixed_joint", [0, 0, 0.2], [0, 0, 0], [0, 0, 1], bounds=(0, 0)),
            URDFLink("joint_4", [0.05, 0, 0], [0, 0, 0], [1, 0, 0], bounds=(-np.pi/2, np.pi/2)),
            URDFLink("joint_5", [0, 0, 0.15], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi)),
            URDFLink("joint_6", [0.05, 0, 0], [0, 0, 0], [1, 0, 0], bounds=(-np.pi/2, np.pi/2)),
            URDFLink("joint_7", [0, 0, 0.15], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi))
        ]
        
        self.my_chain = Chain(name='spear', links=self.links)
        self.q = np.zeros(8)
        self.v_cmd = np.zeros(6)
        
        # Subscribers and publishers
        self.joy_subscription = self.create_subscription(Joy, 'manual/joy2', self.joy_callback, 10)
        self.motor_publisher = self.create_publisher(Float64MultiArray, 'motor_commands', 10)
        
        self.get_logger().info("Inverse Kinematics Node started")
        
    def apply_deadzone(self, value, deadzone=0.15):
        if abs(value) < deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)
    
    def compute_jacobian_improved(self, chain, joint_angles):
        frames = chain.forward_kinematics(joint_angles, full_kinematics=True)
        p_e = np.array(frames[-1][:3, 3])
        
        n_actuated = len([link for link in chain.links[1:] if getattr(link, 'bounds', None) != (0, 0)])
        J = np.zeros((6, n_actuated))
        
        col_idx = 0
        for link_idx, link in enumerate(chain.links[1:], 1):
            if getattr(link, 'bounds', None) == (0, 0):
                continue
                
            frame = frames[link_idx]
            p_i = np.array(frame[:3, 3])
            
            if hasattr(link, 'rotation') and link.rotation is not None:
                rot_axis = np.array(link.rotation)
                z_i = frame[:3, :3].dot(rot_axis)
            else:
                z_i = np.array(frame[:3, 2])
            
            Jv = np.cross(z_i, p_e - p_i)
            Jw = z_i
            J[:, col_idx] = np.concatenate((Jv, Jw))
            col_idx += 1
            
        return J
    
    def publish_motor_commands(self, joint_velocities):
        msg = Float64MultiArray()
        msg.data = joint_velocities.tolist()
        self.motor_publisher.publish(msg)
    
    def joy_callback(self, msg):
        if len(msg.axes) < 6 or len(msg.buttons) < 8:
            return
        
        # Mode toggle
        back_button = msg.buttons[MODE_TOGGLE_BUTTON] if len(msg.buttons) > MODE_TOGGLE_BUTTON else 0
        if back_button == 1 and self.prev_back_button == 0:
            self.joint_mode = not self.joint_mode
            self.get_logger().info(f"Mode: {'Joint' if self.joint_mode else 'Cartesian'}")
        self.prev_back_button = back_button
        
        # Reset
        start_button = msg.buttons[RESET_BUTTON] if len(msg.buttons) > RESET_BUTTON else 0
        if start_button == 1 and self.prev_start_button == 0:
            self.q = np.zeros(8)
            self.publish_motor_commands(np.zeros(7))
        self.prev_start_button = start_button
        
        if self.joint_mode:
            self.process_joint_mode(msg)
        else:
            self.process_cartesian_mode(msg)
    
    def process_joint_mode(self, msg):
        joint_scale = 0.02
        joint_velocities = np.zeros(7)
        
        # D-pad for joint control
        if len(msg.buttons) > max(JOINT1_POS_BUTTON, JOINT1_NEG_BUTTON, JOINT2_POS_BUTTON, JOINT2_NEG_BUTTON):
            if msg.buttons[JOINT1_POS_BUTTON]:
                self.q[1] += joint_scale
                joint_velocities[0] = 1.0
            if msg.buttons[JOINT1_NEG_BUTTON]:
                self.q[1] -= joint_scale
                joint_velocities[0] = -1.0
            if msg.buttons[JOINT2_POS_BUTTON]:
                self.q[2] += joint_scale
                joint_velocities[1] = 1.0
            if msg.buttons[JOINT2_NEG_BUTTON]:
                self.q[2] -= joint_scale
                joint_velocities[1] = -1.0
        
        # Shoulder buttons for joint 4
        if len(msg.buttons) > max(JOINT4_POS_BUTTON, JOINT4_NEG_BUTTON):
            if msg.buttons[JOINT4_POS_BUTTON]:
                self.q[4] += joint_scale
                joint_velocities[3] = 1.0
            if msg.buttons[JOINT4_NEG_BUTTON]:
                self.q[4] -= joint_scale
                joint_velocities[3] = -1.0
        
        # Stick axes for joints 5, 6, 7
        if len(msg.axes) > max(JOINT5_AXIS, JOINT6_AXIS, JOINT7_AXIS):
            joint5_input = self.apply_deadzone(msg.axes[JOINT5_AXIS], self.deadzone)
            joint6_input = self.apply_deadzone(msg.axes[JOINT6_AXIS], self.deadzone)
            joint7_input = self.apply_deadzone(msg.axes[JOINT7_AXIS], self.deadzone)
            
            if abs(joint5_input) > 0.001:
                self.q[5] += joint5_input * joint_scale
                joint_velocities[4] = joint5_input
            if abs(joint6_input) > 0.001:
                self.q[6] += joint6_input * joint_scale
                joint_velocities[5] = joint6_input
            if abs(joint7_input) > 0.001:
                self.q[7] += joint7_input * joint_scale
                joint_velocities[6] = joint7_input
        
        self.publish_motor_commands(joint_velocities)
    
    def process_cartesian_mode(self, msg):
        self.v_cmd = np.zeros(6)
        
        # Movement controls
        if len(msg.axes) > max(MOVE_X_AXIS, MOVE_Y_AXIS):
            move_x = self.apply_deadzone(msg.axes[MOVE_X_AXIS], self.deadzone)
            move_y = self.apply_deadzone(msg.axes[MOVE_Y_AXIS], self.deadzone)
            self.v_cmd[0] = move_x * self.scale
            self.v_cmd[1] = move_y * self.scale
        
        # Orientation controls
        if len(msg.axes) > max(PITCH_AXIS, YAW_AXIS):
            pitch_input = self.apply_deadzone(msg.axes[PITCH_AXIS], self.deadzone)
            yaw_input = self.apply_deadzone(msg.axes[YAW_AXIS], self.deadzone)
            self.v_cmd[4] = pitch_input * self.rotation_scale
            self.v_cmd[5] = yaw_input * self.rotation_scale
        
        # Z movement (triggers)
        if len(msg.axes) > max(MOVE_Z_LEFT_TRIGGER, MOVE_Z_RIGHT_TRIGGER):
            lt = msg.axes[MOVE_Z_LEFT_TRIGGER]
            rt = msg.axes[MOVE_Z_RIGHT_TRIGGER]
            if lt < 0:
                lt = (lt + 1) / 2
            if rt < 0:
                rt = (rt + 1) / 2
            self.v_cmd[2] = (rt - lt) * self.scale
        
        # Roll controls
        if len(msg.buttons) > max(ROLL_LEFT_BUTTON, ROLL_RIGHT_BUTTON):
            if msg.buttons[ROLL_LEFT_BUTTON]:
                self.v_cmd[3] = -self.rotation_scale
            elif msg.buttons[ROLL_RIGHT_BUTTON]:
                self.v_cmd[3] = self.rotation_scale
        
        # Compute IK if there's input
        if np.any(np.abs(self.v_cmd) > 0.001):
            try:
                J = self.compute_jacobian_improved(self.my_chain, self.q.tolist())
                lambda_ = 0.1
                JJT = J.dot(J.T)
                inv = LA.inv(JJT + (lambda_**2) * np.eye(JJT.shape[0]))
                J_pinv = J.T.dot(inv)
                q_dot = J_pinv.dot(self.v_cmd)
                
                joint_velocities = np.zeros(7)
                actuated_idx = 0
                
                for i in range(1, len(self.q)):
                    if i == 3:  # Skip fixed joint
                        continue
                    if actuated_idx < len(q_dot):
                        self.q[i] += q_dot[actuated_idx] * self.dt
                        # Map to joint velocities
                        if i == 1:
                            joint_velocities[0] = q_dot[actuated_idx]
                        elif i == 2:
                            joint_velocities[1] = q_dot[actuated_idx]
                        elif i == 4:
                            joint_velocities[3] = q_dot[actuated_idx]
                        elif i == 5:
                            joint_velocities[4] = q_dot[actuated_idx]
                        elif i == 6:
                            joint_velocities[5] = q_dot[actuated_idx]
                        elif i == 7:
                            joint_velocities[6] = q_dot[actuated_idx]
                        actuated_idx += 1
                
                self.publish_motor_commands(joint_velocities)
                
            except Exception as e:
                self.get_logger().error(f"IK computation error: {str(e)}")
        else:
            self.publish_motor_commands(np.zeros(7))


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = InverseKinematicsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()