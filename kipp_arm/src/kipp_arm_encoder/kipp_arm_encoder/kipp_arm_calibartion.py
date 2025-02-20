import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
import sys
import termios
import tty
import threading
import time

class ArmCalibration(Node):
    def __init__(self):
        super().__init__('arm_calibration')
        
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_velocity_publisher = self.create_publisher(Float64, '/gripper_controller/velocity', 10)
        
        self.calibration_mode = False
        self.known_positions = {
            'joint_1': 0.0,
            'joint_2': 0.0,
            'joint_3': 0.0,
            'joint_4': 0.0,
            'joint_5': 0.0,
            'joint_6': 0.0
        }
        self.offsets = {key: 0.0 for key in self.known_positions.keys()}
        self.relative_positions = {key: 0.0 for key in self.known_positions.keys()}
        
        self.current_joint = 'joint_1'
        self.increment = 0.4
        self.gripper_velocity = 0.2  # Default velocity value for the gripper

        self.gripper_active = False  # Track if the gripper is currently active
        self.gripper_stop_timer = self.create_timer(0.1, self.check_gripper_activity)  # Check every 100 ms

        self.create_subscription(JointState, '/raw/joint_states', self.joint_state_callback, 10)
        
        self.get_logger().info("Press 'c' to toggle calibration mode. Use '1' to '6' to select joints.")
        self.get_logger().info("Use 'i' to increment and 'd' to decrement position.")
        self.get_logger().info("Press 's' to save offsets.")
        self.get_logger().info("Press 'o' to open the gripper and 'p' to close the gripper. Release the key to stop.")
        self.get_logger().info("Use '+' to increase increment value and '-' to decrease it. Current increment: 0.4")
        
        self.keyboard_input()

    def keyboard_input(self):
        tty.setcbreak(sys.stdin)
        while True:
            char = sys.stdin.read(1)
            if char == 'c':
                self.calibration_mode = not self.calibration_mode
                self.get_logger().info(f"Calibration mode {'enabled' if self.calibration_mode else 'disabled'}.")
            elif char in '123456':
                self.current_joint = f'joint_{char}'
                self.get_logger().info(f"Selected {self.current_joint}.")
            elif char == 'i' and self.calibration_mode:
                self.send_incremental_command(self.current_joint, self.increment)
            elif char == 'd' and self.calibration_mode:
                self.send_incremental_command(self.current_joint, -self.increment)
            elif char == 's' and self.calibration_mode:
                self.save_offsets()
            elif char == 'o':  # Open gripper
                self.send_gripper_velocity(self.gripper_velocity)
                self.gripper_active = True
            elif char == 'p':  # Close gripper
                self.send_gripper_velocity(-self.gripper_velocity)
                self.gripper_active = True
            elif char in ['o', 'p']:  # Stop gripper on release
                self.gripper_active = False
            elif char == '+':
                self.increment += 0.1
                self.get_logger().info(f"Increment increased to {self.increment:.2f}")
            elif char == '-':
                self.increment = max(0.1, self.increment - 0.1)
                self.get_logger().info(f"Increment decreased to {self.increment:.2f}")
            elif char == 'q':
                break

    def send_incremental_command(self, joint, increment):
        new_position = self.known_positions[joint] + increment
        self.known_positions[joint] = new_position
        
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = [joint]
        point = JointTrajectoryPoint()
        point.positions = [new_position]
        point.time_from_start.sec = 1
        joint_trajectory.points = [point]
        
        self.joint_trajectory_publisher.publish(joint_trajectory)
        self.get_logger().info(f"Sent command to {joint}: {new_position:.2f} rad")

    def send_gripper_velocity(self, velocity):
        velocity_msg = Float64()
        velocity_msg.data = velocity
        self.gripper_velocity_publisher.publish(velocity_msg)
        self.get_logger().info(f"Sent gripper velocity command: {velocity:.2f}")
        
        time.sleep(0.1) 

        # Immediately send a stop command to ensure the gripper stops
        stop_msg = Float64()
        stop_msg.data = 0.0
        self.gripper_velocity_publisher.publish(stop_msg)
        self.get_logger().info("Sent gripper stop command: 0.0")

    def check_gripper_activity(self):
        self.get_logger().info("Gripper Activity being called")
        if not self.gripper_active:  # If no active command, send 0.0 velocity
            self.send_gripper_velocity(0.0)
        self.gripper_active = False  # Reset the active flag for the next check

    def save_offsets(self):
        self.get_logger().info("Saving offsets...")
        for joint, known_position in self.known_positions.items():
            offset = self.relative_positions[joint] - known_position
            self.offsets[joint] = offset
            self.get_logger().info(f"Offset for {joint}: {offset:.2f} rad")

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.relative_positions:
                self.relative_positions[name] = pos

def main(args=None):
    rclpy.init(args=args)
    arm_calibration_node = ArmCalibration()
    rclpy.spin(arm_calibration_node)
    
    arm_calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()