# Drive Motors
# Front Left: 0x10
# Front Right: 0x11
# Middle Left: 0x12
# Middle Right: 0x13
# Back Left: 0x14
# Back Right: 0x15

# Steering Motors
# Front Left: 0x20
# Front Right: 0x21
# Back Left: 0x22
# Back Right: 0x23

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
import can
import struct
from can.message import Message
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import Joy

class Kipp_Can_Drive(Node):
    def __init__(self):

        super().__init__('kipp_can_drive')
        
        self.cmd_vel_sub = self.create_subscription(Twist, '/manual/cmd_vel', self.cmd_vel_callback, 10)
        self.rotate_sub = self.create_subscription(Joy, '/manual/joy', self.rotate_rover, 10)
        self.timer = self.create_timer(0.25, self.paced_commands)
        self.rotate = 0 
        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate=1000000)

        self.T= 0.455  # Width of the rover (meters)
        self.L1 = 0.325 # front wheels to middle wheels
        self.L2 = 0.425 # back wheels to middle wheels 
        self.v = 0.0
        self.omega = 0.0
        

    def rotate_rover(self, msg):
        self.rotate = msg.buttons[4]


    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x  # Linear velocity (m/s)
        self.omega = msg.angular.z  # Angular velocity (rad/s)


    def paced_commands(self):
        # print(self.rotate)
        if self.rotate != 1:
            steering_angles = self.calculate_steering_angles(self.v, self.omega)

            wheel_velocities = self.calculate_wheel_velocities(self.v, self.omega)

            for actuator_id, angle in steering_angles.items():
                # Convert actuator ID from your custom mapping to actual CAN ID
                can_id = self.steering_actuator_id_to_can_id(actuator_id)
                message = self.create_steering_command(can_id, angle)
                self.bus.send(message)
            
            # Send drive commands to wheel actuators
            for actuator_id, velocity in wheel_velocities.items():
                # Convert actuator ID from your custom mapping to actual CAN ID
                can_id = self.drive_actuator_id_to_can_id(actuator_id)
                message = self.create_drive_command(can_id, velocity)
                self.bus.send(message)
        
        elif self.rotate == 1:
            rotate_angles = self.calculate_rotate_angle()
            wheel_velocities = self.calculate_wheel_velocities(self.v, self.omega)
            
            for actuator_id, angle in rotate_angles.items():
                # Convert actuator ID from your custom mapping to actual CAN ID
                can_id = self.steering_actuator_id_to_can_id(actuator_id)
                message = self.create_steering_command(can_id, angle)
                self.bus.send(message)
            
            for actuator_id, velocity in wheel_velocities.items():
                # Convert actuator ID from your custom mapping to actual CAN ID
                can_id = self.drive_actuator_id_to_can_id(actuator_id)
                message = self.create_drive_command(can_id, velocity)
                self.bus.send(message)
            
    def calculate_rotate_angle(self):

        return {"front_left": -0.880, "front_right": 0.880, "back_left": 1.035, "back_right": -1.035}

           

    def steering_actuator_id_to_can_id(self, actuator_id):
        # Map your actuator_id to its corresponding CAN ID for steering motors
        mapping = {
            "front_left": 0x23, #back_right 
            "front_right": 0x25, #back_left
            "back_left": 0x22, #front_right
            "back_right": 0x24, #front_left
        }
        return mapping.get(actuator_id, None)

    def drive_actuator_id_to_can_id(self, actuator_id):
        # Map your actuator_id to its corresponding CAN ID for drive motors
        mapping = {
            "fl": 0x13,
            "fr": 0x16,
            "ml": 0x12,
            "mr": 0x15,
            "bl": 0x11,
            "br": 0x14,
        }
        return mapping.get(actuator_id, None)

    def calculate_steering_angles(self, v, omega):
        """
        Calculate wheel velocities for a 6-wheel rover with differential speeds.
        
        Parameters:
        v (float): Linear velocity of the rover (m/s)
        omega (float): Angular velocity of the rover (rad/s)
        
        Returns:
        dict: Wheel velocities (m/s) for each wheel (fl, fr, ml, mr, bl, br)
        """
        
        if omega == 0:
            return {"front_left": 0, "front_right": 0, "back_left": 0, "back_right": 0}

        

        v = 1
        if omega != 0: 
            r = v / omega
            
            front_left_rad = math.atan((self.L1)/(r - (self.T)/2))
            front_right_rad = math.atan((self.L1)/(r + (self.T)/2))
            back_left_rad = -(math.atan((self.L2)/(r - (self.T)/2)))
            back_right_rad = -(math.atan((self.L2)/(r + (self.T)/2)))
            angles = {
                "front_left": front_left_rad, "front_right": front_right_rad,
                "back_left": back_left_rad, "back_right": back_right_rad  
            }

            return angles
        return {"front_left": 0, "front_right": 0, "back_left": 0, "back_right": 0}
        
        
        

    def calculate_wheel_velocities(self, v, omega):
        """
        Calculate individual wheel velocities for a 6-wheel rover to mimic a differential drive.
        The rover has 3 wheels per side:
        - Front left:  (self.L1,  self.T/2)
        - Middle left: (0,           self.T/2)
        - Back left:   (-self.L2,  self.T/2)
        - Front right: (self.L1, -self.T/2)
        - Middle right:(0,         -self.T/2)
        - Back right:  (-self.L2, -self.T/2)

        For combined translation and rotation (v != 0 and ω != 0), the turning center is at (0, R) with R = v/ω.
        Each wheel’s effective turning radius is:
            r_eff = sqrt( x^2 + (y - R)^2 )
        and its speed is then:
            v_wheel = sign(v) * |ω| * r_eff

        In a straight line (ω ≈ 0), all wheels get the same speed v.
        In pure rotation (v ≈ 0 but ω ≠ 0), we approximate differential speeds based on track width.
        """
        # Limit the commanded linear velocity
        velocity_limiter = 0.5
        if v > velocity_limiter:
            v = velocity_limiter
        elif v < -velocity_limiter:
            v = -velocity_limiter

        # Define the wheel positions relative to the robot center (x forward, y left)
        wheels = {
            "fl": (self.L1,  self.T / 2),   # Front Left
            "ml": (0.0,       self.T / 2),   # Middle Left
            "bl": (-self.L2,  self.T / 2),   # Back Left
            "fr": (self.L1, -self.T / 2),   # Front Right
            "mr": (0.0,      -self.T / 2),   # Middle Right
            "br": (-self.L2, -self.T / 2)    # Back Right
        }

        speeds = {}
        # Check for straight-line motion
        if abs(omega) < 1e-6:
            for key in wheels:
                speeds[key] = v
        # Check for pure rotation (almost zero forward velocity)
        elif abs(v) < 1e-3:
            # For pure rotation, a common differential-drive approximation:
            # Left wheels:  v = -|ω|*(T/2)
            # Right wheels: v =  |ω|*(T/2)
            for key, (x, y) in wheels.items():
                if y > 0:  # left side wheels
                    speeds[key] = -abs(omega) * (self.T / 2)
                else:      # right side wheels
                    speeds[key] =  abs(omega) * (self.T / 2)
        else:
            # Combined translation and rotation:
            # Compute turning radius of the robot's center
            R = v / omega
            v_sign = 1 if v >= 0 else -1
            for key, (x, y) in wheels.items():
                # Compute the effective radius from the turning center at (0, R)
                # (This works whether R is positive (left turn) or negative (right turn).)
                r_eff = math.sqrt(x * x + (y - R) ** 2)
                speeds[key] = v_sign * abs(omega) * r_eff
        return speeds

    
    def create_drive_command(self, actuator_id, velocity = 0):
        # Construct arbitration ID
        priority=0x0
        command_id=0x03 
        receiver_node_id=actuator_id
        sender_node_id=1
        arbitration_id = priority << 24 | command_id << 16 | receiver_node_id << 8 | sender_node_id
        print(f"Velocity: {velocity} ID {actuator_id}")
        # Pack data (velocity)
        data = struct.pack(">f", velocity)  # 'f' for float32
        # Create CAN message
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        
        return message

    def create_steering_command(self, actuator_id, angle = 0):
        # Construct arbitration ID
        priority=0x0
        command_id=0x02 
        receiver_node_id=actuator_id
        sender_node_id=1
        arbitration_id = priority << 24 | command_id << 16 | receiver_node_id << 8 | sender_node_id
        print(f"angle: {180*angle/math.pi} ID {actuator_id}")
        #if actuator_id == 36:
        #    print(f"angle: {180*angle/math.pi} ID {actuator_id}")
        
        
        # Pack data (angle)
        data = struct.pack(">f", angle)  # 'f' for float32
        
        # Create CAN message
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        
        return message
    

def main(args=None):

    rclpy.init(args=args)

    can_node = Kipp_Can_Drive()

    rclpy.spin(can_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

