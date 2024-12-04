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


drive_topic = "/Rover/drive_commands"
steer_topic = "/Rover/steer_commands"
joy_topic = "/Rover/Joy_Topic"

LEFT_STICK_X = 0
LEFT_STICK_Y = 1
LEFT_TRIGGER = 2
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
RIGHT_TRIGGER = 5
D_PAD_X = 6
D_PAD_Y = 7
A = 0
B = 1
X = 2
Y = 3
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
CHANGE_VIEW = 6
MENU = 7
HOME = 8
LEFT_STICK_CLICK = 9
RIGHT_STICK_CLICK = 10

#Note Motor Mapping 

"""
0 3
1 4
2 5
"""

"""
Steering Motor Map 

0 1 
2 3
"""

class RoverControl(Node):

    def __init__(self):
        super().__init__('rover_control')

        self.joystick_msg_timestamp = time.time()

        #Publsiher drive and steer commands
        self.drive_cmds = self.create_publisher(msg_type = Float32MultiArray, topic = drive_topic, qos_profile = QoSProfile(depth=10))
        self.steer_cmds = self.create_publisher(msg_type = Float32MultiArray, topic = steer_topic, qos_profile = QoSProfile(depth=10))

        #Subscription to Joy commands         
        self.joy_sub = self.create_subscription(msg_type = Joy, topic = joy_topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.JoystickMsg)

        
        timer_period = 0.1

        self.last_toggle_time = time.time()
        self.steer_lock_state = True
        self.toggle_debounce_time = 0.5 #debounce time
        self.drive_motors_num = 6 #Number of drive motors
        self.steer_motors_num = 4 #Number of steering motors

        self.steer_right_limit = -1.0472
        self.steer_left_limit = 1.0472
        self.steer_center = (self.steer_right_limit + self.steer_left_limit)/2

        
        
        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.rover_main_control)

        self.joystick_msg = Joy()

    def JoystickMsg(self, msg):
        
        self.joystick_msg = msg
        self.joystick_msg_timestamp = Time.from_msg(msg.header.stamp)
    
    def rover_main_control(self):
        
        timeout = 1.0  # Set the timeout value, for example 1 second
        
        # Terminate the program if no new joystick message has been received within the timeout
        if self.get_clock().now() - self.joystick_msg_timestamp > rclpy.duration.Duration(seconds=timeout):
            self.get_logger().error("Joystick timeout. Ending program.")
            rclpy.shutdown()
            sys.exit(1)  # This will terminate the program


        self.drive_cmds_msg = Float32MultiArray()
        self.steer_cmds_msg = Float32MultiArray()

        #Number of Drive Motors and Steering Motors
        
        self.drive_cmds_msg.data = [0.0]* self.drive_motors_num
        self.steer_cmds_msg.data = [self.steer_center]* self.steer_motors_num

        if self.joystick_msg.buttons[RIGHT_BUMPER] or self.joystick_msg.buttons[LEFT_BUMPER]:

            self.skid_steer_mode()
            self.get_logger().warn("CRITICAL: Steer message data array doesn't have enough elements.")

        else:
            
            self.drive_motor_conv()

            if self.steer_lock_state:

                self.steer_motor_conv_locked()

            elif not self.steer_lock_state:

                self.steer_motor_conv_unlocked()
        
        print(self.steer_lock_state)
        if(self.joystick_msg.buttons[MENU]):
            self.steer_toggle()

        self.drive_cmds.publish(self.drive_cmds_msg)

        self.steer_cmds.publish(self.steer_cmds_msg)




    def drive_motor_conv(self):

        threshold = 0.0
        num_motors = 6
        

        if self.joystick_msg.axes[RIGHT_TRIGGER] != -1.0:
            

            value = self.trigger_map(self.joystick_msg.axes[RIGHT_TRIGGER])

            self.drive_cmds_msg.data = [value] * self.drive_motors_num
            
        
        elif self.joystick_msg.axes[LEFT_TRIGGER] != -1.0:

            
            value = -(self.trigger_map(self.joystick_msg.axes[LEFT_TRIGGER]))
            
            self.drive_cmds_msg.data = [value] * self.drive_motors_num

        else:
            
            self.drive_cmds_msg.data = [0.0] * self.drive_motors_num
            


    def steer_motor_conv_unlocked(self):

        """
        Take steer comands from RS and LS and populate steer_cmds_msg

        Implment such that the wheels can't turn in oppistie directions 
        """
        threshold = 0.2

        if self.joystick_msg.axes[LEFT_STICK_X] > threshold or self.joystick_msg.axes[LEFT_STICK_X] < -threshold:

            self.steer_cmds_msg.data[0] = self.map_value(self.joystick_msg.axes[LEFT_STICK_X])
            self.steer_cmds_msg.data[1] = self.map_value(self.joystick_msg.axes[LEFT_STICK_X])
        

        if self.joystick_msg.axes[RIGHT_STICK_X] > threshold or self.joystick_msg.axes[RIGHT_STICK_X] < -threshold:

            self.steer_cmds_msg.data[2] = -(self.map_value(self.joystick_msg.axes[RIGHT_STICK_X]))
            self.steer_cmds_msg.data[3] = -(self.map_value(self.joystick_msg.axes[RIGHT_STICK_X]))

        

    def steer_motor_conv_locked(self):

        """
        Take steer commands from only LS ignore RS and populate steer_cmds_msg
        """

        threshold = 0.2

        if self.joystick_msg.axes[LEFT_STICK_X] > threshold or self.joystick_msg.axes[LEFT_STICK_X] < -threshold:

            print(self.map_value(self.joystick_msg.axes[LEFT_STICK_X]))

            self.steer_cmds_msg.data[0] = self.map_value(self.joystick_msg.axes[LEFT_STICK_X])
            self.steer_cmds_msg.data[1] = self.map_value(self.joystick_msg.axes[LEFT_STICK_X])
            self.steer_cmds_msg.data[2] = -(self.map_value(self.joystick_msg.axes[LEFT_STICK_X]))
            self.steer_cmds_msg.data[3] = -(self.map_value(self.joystick_msg.axes[LEFT_STICK_X]))

        

    def skid_steer_mode(self):

        skid_steer_speed = 0.4

        if self.joystick_msg.buttons[LEFT_BUMPER]:

            self.drive_cmds_msg.data[0] = -0.5
            self.drive_cmds_msg.data[1] = -0.5
            self.drive_cmds_msg.data[2] = -0.5
            self.drive_cmds_msg.data[3] = -0.3
            self.drive_cmds_msg.data[4] = -0.3
            self.drive_cmds_msg.data[5] = -0.3

        elif self.joystick_msg.buttons[RIGHT_BUMPER]:

            self.drive_cmds_msg.data[0] = -0.3
            self.drive_cmds_msg.data[1] = -0.3
            self.drive_cmds_msg.data[2] = -0.3
            self.drive_cmds_msg.data[3] = -0.5
            self.drive_cmds_msg.data[4] = -0.5
            self.drive_cmds_msg.data[5] = -0.5


    def steer_toggle(self):

        current_time = time.time()
        if current_time - self.last_toggle_time > self.toggle_debounce_time:
            self.steer_lock_state = not self.steer_lock_state
            self.last_toggle_time = current_time

    def map_value(self, value):
        
        return float((value + 1) * (self.steer_left_limit - self.steer_right_limit) / (2) + self.steer_right_limit)

    def trigger_map(self, value):

        x_min = -1.0
        x_max = 1.0
        y_min = 0.00
        y_max = 1.0

        return y_min + (value - x_min) * (y_max - y_min) / (x_max - x_min)
        
    

def main(args=None):
    rclpy.init(args=args)

   
    SPEAR_Rover_Node = RoverControl()

    rclpy.spin(SPEAR_Rover_Node)

    SPEAR_Rover_Node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()