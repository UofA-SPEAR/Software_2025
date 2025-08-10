#!/usr/bin/env python3

import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from numpy import linalg as LA
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Joint Mode Controller Binds (matching CAN actuator mapping)
        self.joint0_pos_bind = 0      # A button  - 0x31 Shoulder Yaw
        self.joint0_neg_bind = 1      # B button  - 0x31 Shoulder Yaw
        self.joint1_pos_bind = 2      # X button  - 0x32 Shoulder Pitch
        self.joint1_neg_bind = 3      # Y button  - 0x32 Shoulder Pitch
        self.joint2_pos_bind = 4      # LB button - 0x34 Elbow Pitch
        self.joint2_neg_bind = 5      # RB button - 0x34 Elbow Pitch
        self.joint3_axis_bind = 0     # Left stick X  - 0x35 Elbow Roll
        self.joint4_axis_bind = 1     # Left stick Y  - 0x36 Wrist Pitch
        self.joint5_axis_bind = 3     # Right stick X - 0x16 Wrist Roll
        # Joint 6 (0x15 End Effector) - Not controlled in joint mode
        
        self.joint_mode = False
        self.scale = 5.0
        self.rotation_scale = 25.0
        self.dt = 0.01
        self.deadzone = 0.15
        
        self.prev_back_button = 0
        self.prev_start_button = 0
        
        self.links = [
            OriginLink(),
            URDFLink("base_rotation", [0, 0, 0.12], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi)),
            URDFLink("shoulder_lift", [0, -0.06, 0], [0, 0, 0], [0, 1, 0], bounds=(-np.pi/2, np.pi/2)),
            URDFLink("fixed_joint", [0, 0, 0.35], [0, 0, 0], [0, 0, 1], bounds=(0, 0)),
            URDFLink("joint_4", [0, 0.0105, 0], [0, 0, 0], [0, 1, 0], bounds=(-np.pi/2, np.pi/2)),
            URDFLink("joint_5", [0, 0, 0.275], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi)),
            URDFLink("joint_6", [0, 0.065, 0], [0, 0, 0], [0, 1, 0], bounds=(-np.pi/2, np.pi/2)),
            URDFLink("joint_7", [0, 0, 0.375], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi))
        ]
        
        self.my_chain = Chain(name='spear', links=self.links)
        self.q = np.zeros(8)
        self.v_cmd = np.zeros(6)
        
        self.joy_subscription = self.create_subscription(Joy, 'manual/joy2', self.joy_callback, 10)
        self.encoder_subscription = self.create_subscription(Float64MultiArray, 'arm_joint_angles', self.encoder_callback, 10)
        self.motor_publisher = self.create_publisher(Float64MultiArray, 'motor_commands', 10)
        
    def encoder_callback(self, msg):
        if len(msg.data) >= 6:
            self.q[1] = msg.data[0]
            self.q[2] = msg.data[1]
            self.q[4] = msg.data[2]
            self.q[5] = msg.data[3]
            self.q[6] = msg.data[4]
            self.q[7] = msg.data[5]
    
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
        
        back_button = msg.buttons[6] if len(msg.buttons) > 6 else 0
        if back_button == 1 and self.prev_back_button == 0:
            self.joint_mode = not self.joint_mode
        self.prev_back_button = back_button
        
        start_button = msg.buttons[7] if len(msg.buttons) > 7 else 0
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
        
        # Joint 0 (0x31 - Shoulder Yaw)
        if msg.buttons[self.joint0_pos_bind]:
            self.q[1] += joint_scale
            joint_velocities[0] = 1.0
        if msg.buttons[self.joint0_neg_bind]:
            self.q[1] -= joint_scale
            joint_velocities[0] = -1.0
            
        # Joint 1 (0x32 - Shoulder Pitch)
        if msg.buttons[self.joint1_pos_bind]:
            self.q[2] += joint_scale
            joint_velocities[1] = 1.0
        if msg.buttons[self.joint1_neg_bind]:
            self.q[2] -= joint_scale
            joint_velocities[1] = -1.0
            
        # Joint 2 (0x34 - Elbow Pitch)
        if msg.buttons[self.joint2_pos_bind]:
            self.q[4] += joint_scale
            joint_velocities[2] = 1.0
        if msg.buttons[self.joint2_neg_bind]:
            self.q[4] -= joint_scale
            joint_velocities[2] = -1.0
        
        # Joint 3 (0x35 - Elbow Roll) - axis control
        joint3_input = self.apply_deadzone(msg.axes[self.joint3_axis_bind], self.deadzone)
        if abs(joint3_input) > 0.001:
            self.q[5] += joint3_input * joint_scale
            joint_velocities[3] = joint3_input
            
        # Joint 4 (0x36 - Wrist Pitch) - axis control
        joint4_input = self.apply_deadzone(msg.axes[self.joint4_axis_bind], self.deadzone)
        if abs(joint4_input) > 0.001:
            self.q[6] += joint4_input * joint_scale
            joint_velocities[4] = joint4_input
            
        # Joint 5 (0x16 - Wrist Roll) - axis control
        joint5_input = self.apply_deadzone(msg.axes[self.joint5_axis_bind], self.deadzone)
        if abs(joint5_input) > 0.001:
            self.q[7] += joint5_input * joint_scale
            joint_velocities[5] = joint5_input
        
        # Joint 6 (0x15 - End Effector) - NOT CONTROLLED IN JOINT MODE
        # joint_velocities[6] remains 0
        
        self.publish_motor_commands(joint_velocities)
    
    def process_cartesian_mode(self, msg):
        self.v_cmd = np.zeros(6)
        
        move_x = self.apply_deadzone(msg.axes[0], self.deadzone)
        move_y = self.apply_deadzone(msg.axes[1], self.deadzone)
        self.v_cmd[0] = move_x * self.scale
        self.v_cmd[1] = move_y * self.scale
        
        pitch_input = self.apply_deadzone(msg.axes[3], self.deadzone)
        yaw_input = self.apply_deadzone(msg.axes[4], self.deadzone)
        self.v_cmd[4] = pitch_input * self.rotation_scale
        self.v_cmd[5] = yaw_input * self.rotation_scale
        
        lt = msg.axes[2]
        rt = msg.axes[5]
        if lt < 0:
            lt = (lt + 1) / 2
        if rt < 0:
            rt = (rt + 1) / 2
        self.v_cmd[2] = (rt - lt) * self.scale
        
        if msg.buttons[4]:
            self.v_cmd[3] = -self.rotation_scale
        elif msg.buttons[5]:
            self.v_cmd[3] = self.rotation_scale
        
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
                    if i == 3:
                        continue
                    if actuated_idx < len(q_dot):
                        self.q[i] += q_dot[actuated_idx] * self.dt
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