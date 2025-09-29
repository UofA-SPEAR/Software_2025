#!/usr/bin/env python3

import warnings
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from numpy import linalg as LA
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

warnings.filterwarnings("ignore", message="Link Base link.*")

# Configure matplotlib
import matplotlib as mpl
mpl.use('Qt5Agg')
for key in ['save', 'fullscreen', 'quit', 'grid', 'home', 'back', 'forward', 'help', 'yscale', 'xscale']:
    mpl.rcParams[f'keymap.{key}'] = []

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Control configuration
        self.joint_mode = False
        self.scale = 5.0
        self.rotation_scale = 25.0  # Separate scale for rotations
        self.dt = 0.01
        self.updating_sliders = False
        self.deadzone = 0.15  # Joystick deadzone
        
        # Previous button states for debouncing
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
        
        # Create subscriber to joy topic
        self.joy_subscription = self.create_subscription(
            Joy,
            'manual/joy2',
            self.joy_callback,
            10)
        
        # Create publisher for motor commands
        self.motor_publisher = self.create_publisher(
            Float64MultiArray,
            'motor_commands',
            10)
        
        self.get_logger().info("Inverse Kinematics Node started. Waiting for joy messages on 'manual/joy2'...")
        
        # Setup GUI
        self.setup_gui()
        
    def apply_deadzone(self, value, deadzone=0.15):
        """Apply deadzone to joystick input"""
        if abs(value) < deadzone:
            return 0.0
        # Scale the remaining range to 0-1
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
        """Publish joint velocities to motors"""
        msg = Float64MultiArray()
        msg.data = joint_velocities.tolist()
        self.motor_publisher.publish(msg)
    
    def setup_gui(self):
        """Setup the matplotlib GUI"""
        self.fig = plt.figure(figsize=(10, 8))
        self.fig.subplots_adjust(left=0.15, right=0.78, bottom=0.1, top=0.9)
        self.ax = self.fig.add_axes([0.15, 0.3, 0.55, 0.65], projection='3d')
        self.ax_xy = self.fig.add_axes([0.79, 0.55, 0.18, 0.35])
        
        self.ax_xy.set_title('XY View')
        self.ax_xy.set_xlim(-0.5, 0.5)
        self.ax_xy.set_ylim(-0.5, 0.5)
        self.ax_xy.set_aspect('equal')
        self.ax_xy.grid(True)
        
        # Add controller instructions
        self.fig.text(0.02, 0.95, "Xbox Controller (ROS2):", fontsize=12, fontweight='bold')
        self.fig.text(0.02, 0.90, "Left Stick (0,1): Move X/Y", fontsize=10)
        self.fig.text(0.02, 0.86, "Right Stick (2,3): Pitch/Yaw", fontsize=10)
        self.fig.text(0.02, 0.82, "Triggers (4,5): Move Up/Down", fontsize=10)
        self.fig.text(0.02, 0.78, "LB/RB: Roll Left/Right", fontsize=10)
        self.fig.text(0.02, 0.74, "Back: Toggle Mode", fontsize=10)
        self.fig.text(0.02, 0.70, "Start: Reset Pose", fontsize=10)
        self.fig.text(0.02, 0.66, f"Topic: manual/joy2", fontsize=10, style='italic')
        self.fig.text(0.02, 0.62, f"Motor Topic: motor_commands", fontsize=10, style='italic')
        
        # Create sliders
        self.sliders = []
        for i in range(7):
            ax_sl = self.fig.add_axes([0.02, 0.1 + (6-i)*0.08, 0.12, 0.06])
            slider = Slider(ax_sl, f'J{i+1}', -np.pi, np.pi, valinit=self.q[i+1])
            slider.on_changed(lambda val, idx=i: self.update_joint(idx, val))
            self.sliders.append(slider)
        
        # Initial plot
        self.plot_arm(self.q)
        plt.ion()
        plt.show(block=False)
    
    def update_joint(self, idx, val):
        """Update joint from slider"""
        if self.updating_sliders:
            return
        self.q[idx+1] = val
        self.plot_arm(self.q)
    
    def update_all_sliders(self):
        """Update all sliders to match current joint angles"""
        self.updating_sliders = True
        try:
            for i, slider in enumerate(self.sliders):
                if i+1 < len(self.q):
                    slider.set_val(self.q[i+1])
        finally:
            self.updating_sliders = False
    
    def plot_arm(self, joint_angles):
        """Plot the robot arm"""
        frames = self.my_chain.forward_kinematics(joint_angles, full_kinematics=True)
        pts = [f[:3, 3] for f in frames]
        X, Y, Z = [p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]
        
        self.ax.clear()
        self.ax.plot3D(X, Y, Z, linewidth=4, color='gray')
        self.ax.scatter(X, Y, Z, s=60, color='steelblue', depthshade=True)
        self.ax.scatter([pts[-1][0]], [pts[-1][1]], [pts[-1][2]], marker='x', color='red', s=80)
        
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(0.0, 0.5)
        
        xy_pts = np.array(pts)
        self.ax_xy.clear()
        self.ax_xy.plot(xy_pts[:, 0], xy_pts[:, 1], '-o', color='gray', linewidth=2, markersize=4)
        self.ax_xy.scatter(xy_pts[-1, 0], xy_pts[-1, 1], color='red', s=50)
        self.ax_xy.set_xlim(-0.5, 0.5)
        self.ax_xy.set_ylim(-0.5, 0.5)
        self.ax_xy.set_aspect('equal')
        self.ax_xy.grid(True)
        
        # Update the plot
        try:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except:
            pass  # Ignore drawing errors
    
    def joy_callback(self, msg):
        """Process ROS2 Joy messages from Xbox controller"""
        try:
            # Ensure we have enough axes and buttons
            if len(msg.axes) < 6 or len(msg.buttons) < 8:
                self.get_logger().warn(f"Insufficient axes ({len(msg.axes)}) or buttons ({len(msg.buttons)})")
                return
            
            # Button debouncing for mode toggle (Back button - typically button 6)
            back_button = msg.buttons[6] if len(msg.buttons) > 6 else 0
            if back_button == 1 and self.prev_back_button == 0:
                self.joint_mode = not self.joint_mode
                self.get_logger().info(f"Mode: {'Joint' if self.joint_mode else 'Cartesian'}")
            self.prev_back_button = back_button
            
            # Button debouncing for reset (Start button - typically button 7)
            start_button = msg.buttons[7] if len(msg.buttons) > 7 else 0
            if start_button == 1 and self.prev_start_button == 0:
                self.q = np.zeros(8)
                self.plot_arm(self.q)
                self.update_all_sliders()
                self.get_logger().info("Robot pose reset")
                # Publish zero velocities on reset
                self.publish_motor_commands(np.zeros(7))
            self.prev_start_button = start_button
            
            if self.joint_mode:
                self.process_joint_mode(msg)
            else:
                self.process_cartesian_mode(msg)
                
        except Exception as e:
            self.get_logger().error(f"Error in joy callback: {str(e)}")
    
    def process_joint_mode(self, msg):
        """Process joint mode control using D-pad and buttons"""
        joint_scale = 0.02
        joint_velocities = np.zeros(7)
        
        # Use buttons for joint control (A, B, X, Y, LB, RB)
        if len(msg.buttons) >= 6:
            # A button (usually index 0) - Joint 1 positive
            if msg.buttons[0]:
                self.q[1] += joint_scale
                joint_velocities[0] = 1.0
            # B button (usually index 1) - Joint 1 negative  
            if msg.buttons[1]:
                self.q[1] -= joint_scale
                joint_velocities[0] = -1.0
            # X button (usually index 2) - Joint 2 positive
            if msg.buttons[2]:
                self.q[2] += joint_scale
                joint_velocities[1] = 1.0
            # Y button (usually index 3) - Joint 2 negative
            if msg.buttons[3]:
                self.q[2] -= joint_scale
                joint_velocities[1] = -1.0
            # LB (usually index 4) - Joint 4 positive
            if msg.buttons[4]:
                self.q[4] += joint_scale
                joint_velocities[3] = 1.0
            # RB (usually index 5) - Joint 4 negative
            if msg.buttons[5]:
                self.q[4] -= joint_scale
                joint_velocities[3] = -1.0
        
        # Use stick axes for other joints
        if len(msg.axes) >= 4:
            # Left stick for joints 5 and 6
            left_x = self.apply_deadzone(msg.axes[0], self.deadzone)
            left_y = self.apply_deadzone(-msg.axes[1], self.deadzone)  # Inverted
            
            if abs(left_x) > 0.001:
                self.q[5] += left_x * joint_scale
                joint_velocities[4] = left_x
            if abs(left_y) > 0.001:
                self.q[6] += left_y * joint_scale
                joint_velocities[5] = left_y
        
        self.plot_arm(self.q)
        self.update_all_sliders()
        self.publish_motor_commands(joint_velocities)
    
    def process_cartesian_mode(self, msg):
        """Process Cartesian mode control for full 6DOF"""
        self.v_cmd = np.zeros(6)
        
        # Left stick: X/Y movement (axes 0 and 1)
        if len(msg.axes) >= 2:
            left_x = self.apply_deadzone(msg.axes[0], self.deadzone)   # Left/Right
            left_y = self.apply_deadzone(-msg.axes[1], self.deadzone) # Forward/Back (inverted)
            self.v_cmd[0] = left_x * self.scale           # X velocity
            self.v_cmd[1] = left_y * self.scale           # Y velocity
        
        # Right stick: Pitch/Yaw orientation (axes 2 and 3)  
        if len(msg.axes) >= 4:
            right_x = self.apply_deadzone(msg.axes[2], self.deadzone)  # Yaw (around Z)
            right_y = self.apply_deadzone(-msg.axes[3], self.deadzone) # Pitch (around Y, inverted)
            self.v_cmd[4] = right_y * self.rotation_scale   # Pitch (around Y)
            self.v_cmd[5] = right_x * self.rotation_scale   # Yaw (around Z)
        
        # Triggers: Z movement (axes 4 and 5)
        # Note: Trigger values in ROS Joy messages are typically -1 to 1
        if len(msg.axes) >= 6:
            lt = msg.axes[4]  # Left trigger 
            rt = msg.axes[5]  # Right trigger
            # Convert from -1..1 range to 0..1 range if needed
            if lt < 0:
                lt = (lt + 1) / 2
            if rt < 0:
                rt = (rt + 1) / 2
            z_vel = rt - lt  # Up when RT pressed, down when LT pressed
            self.v_cmd[2] = z_vel * self.scale            # Z velocity
        
        # Shoulder buttons: Roll (buttons 4 and 5 are typically LB/RB)
        roll_vel = 0
        if len(msg.buttons) >= 6:
            if msg.buttons[4]:  # Left bumper (roll left)
                roll_vel = -1.0
            elif msg.buttons[5]:  # Right bumper (roll right)
                roll_vel = 1.0
        self.v_cmd[3] = roll_vel * self.rotation_scale  # Roll (around X)
        
        # Only compute IK if there's actual input
        if np.any(np.abs(self.v_cmd) > 0.001):
            try:
                J = self.compute_jacobian_improved(self.my_chain, self.q.tolist())
                sv = LA.svd(J, compute_uv=False)
                if sv.min() < 0.01:
                    self.get_logger().warn(f"Near singularity: {sv.min():.4f}")
                
                lambda_ = 0.1
                JJT = J.dot(J.T)
                inv = LA.inv(JJT + (lambda_**2) * np.eye(JJT.shape[0]))
                J_pinv = J.T.dot(inv)
                q_dot = J_pinv.dot(self.v_cmd)
                
                # Create joint velocities array for publishing (7 joints)
                joint_velocities = np.zeros(7)
                actuated_idx = 0
                
                for i in range(1, len(self.q)):
                    if i == 3:  # Skip fixed joint
                        continue
                    if actuated_idx < len(q_dot):
                        self.q[i] += q_dot[actuated_idx] * self.dt
                        # Map to joint velocities for motor commands
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
                
                self.plot_arm(self.q)
                self.update_all_sliders()
                self.publish_motor_commands(joint_velocities)
                
            except Exception as e:
                self.get_logger().error(f"IK computation error: {str(e)}")
        else:
            # Publish zero velocities when no input
            self.publish_motor_commands(np.zeros(7))


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = InverseKinematicsNode()
        
        # Use a timer to keep matplotlib responsive
        def update_plot():
            try:
                plt.pause(0.001)  # Small pause to allow matplotlib to update
            except:
                pass
        
        # Create a timer to update the plot
        timer = node.create_timer(0.05, update_plot)  # 20Hz update rate
        
        node.get_logger().info("Starting ROS2 spinning...")
        
        # Keep both ROS2 and matplotlib running
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Shutting down...")
        finally:
            node.destroy_node()
            rclpy.shutdown()
            plt.close('all')
            
    except Exception as e:
        print(f"Failed to start node: {e}")
        rclpy.shutdown()


if __name__ == '__main__':
    main()