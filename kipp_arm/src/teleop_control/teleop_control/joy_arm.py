import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

Joy_Topic = "/kipps_arm/Joy_Topic"
Twist_Topic = "/kipps_arm/delta_twist_cmds"
Joint_Topic = "/kipps_arm/delta_joint_cmds"
EEF_Frame_ID = "end_effector"
BASE_FRAME_ID = "base_link"
EEF_Topic = "/kipps_arm/EEF"


# Initalize joystick commands
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


Axis_Default = {

    "LEFT_TRIGGER" : 1.0,
    "RIGHT_TRIGGER" : 1.0,
}

class Arm_Control(Node):

    def __init__(self):
        super().__init__('Arm_Control')
       
        self.frame_to_publish_ = EEF_Frame_ID

        

        self.twist_pub = self.create_publisher(msg_type = TwistStamped, topic = Twist_Topic, qos_profile = QoSProfile(depth=10))
        self.joint_pub = self.create_publisher(msg_type = JointJog, topic = Joint_Topic, qos_profile = QoSProfile(depth=10))
        self.publisher_ = self.create_publisher(Float32, topic=EEF_Topic, qos_profile=QoSProfile(depth=10))


        
        self.joy_sub = self.create_subscription(msg_type = Joy, topic = Joy_Topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.JoystickMsg)
        
        
        timer_period = 0.5

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.JoyMain)
        
        self.joystick_msg = Joy()


    def JoyMain(self):

        self.twist_msg = TwistStamped()
        self.joint_msg = JointJog()

        ik = self.ConvertJoyToCommand()
        
        if ik:

            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_msg.header.frame_id = "base_link"
            self.twist_pub.publish(self.twist_msg)


        else:
            
            self.joint_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_msg.header.frame_id = "base_link"
            self.joint_pub.publish(self.joint_msg)


    def ConvertJoyToCommand(self):


        i = 0
        if (abs(self.joystick_msg.axes[RIGHT_STICK_X]) > 0.1 and self.joystick_msg.buttons[A]):
            # Shoulder Roll
            i+=1
            self.joint_msg.joint_names.append("base_joint")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

       
        if (abs(self.joystick_msg.axes[RIGHT_STICK_X]) > 0.1 and self.joystick_msg.buttons[B]):
            i+=1
            # Shoulder pitch
            self.joint_msg.joint_names.append("link1_joint")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

       
        if (abs(self.joystick_msg.axes[RIGHT_STICK_X]) > 0.1 and self.joystick_msg.buttons[X]):
            i+=1
            # Elbow Roll
            self.joint_msg.joint_names.append("link2_joint")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

       
        if (abs(self.joystick_msg.axes[RIGHT_STICK_X]) > 0.1 and self.joystick_msg.buttons[Y]):
            i+=1
            # Elbow Pitch
            self.joint_msg.joint_names.append("link3_joint")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])
            
       
        if (abs(self.joystick_msg.axes[RIGHT_STICK_X]) > 0.1 and self.joystick_msg.buttons[CHANGE_VIEW]):
            i+=1
            # Wrist Roll
            self.joint_msg.joint_names.append("link4_joint")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

        
        if (abs(self.joystick_msg.axes[RIGHT_STICK_X]) > 0.1 and self.joystick_msg.buttons[MENU]):
            i+=1
            # Wrist Pitch
            self.joint_msg.joint_names.append("eef_joint")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

        if self.joystick_msg.buttons[RIGHT_BUMPER]:
            i+=1
            # EEF
            eef_open_msg = Float32()
            eef_open_msg.data = self.joystick_msg.axes[RIGHT_BUMPER]
            self.publisher_.publish(eef_open_msg)
        
        if self.joystick_msg.buttons[RIGHT_TRIGGER]:
            i+=1
            # EEF
            eef_close_msg = Float32()
            eef_close_msg.data = -self.joystick_msg.axes[RIGHT_TRIGGER] # You can set the value for EEF Close as -1.0 or any other value as per your requirement
            self.publisher_.publish(eef_close_msg)
        
        if i > 0:
            return False
        
        else:
            # Linear IK
            self.twist_msg.twist.linear.z = (self.joystick_msg.axes[LEFT_STICK_Y])
            self.twist_msg.twist.linear.y = self.joystick_msg.axes[LEFT_STICK_X]         
            # When the left bumper is pressed, move back along the x-axis
            lin_x_left_bumper = -0.5 * (self.joystick_msg.buttons[LEFT_BUMPER] - 0)
            # When the right trigger is pressed, move forward along the x-axis
            lin_x_right_trigger = 0.5 * (self.joystick_msg.axes[RIGHT_TRIGGER] - Axis_Default["RIGHT_TRIGGER"])
            # Combine the two values for the final x velocity
            self.twist_msg.twist.linear.x = lin_x_left_bumper + lin_x_right_trigger

            # Angular IK
            self.twist_msg.twist.angular.y = self.joystick_msg.axes[D_PAD_Y]
            self.twist_msg.twist.angular.x = self.joystick_msg.axes[D_PAD_Y]
            # Get the Y-axis value from the right stick (assuming positive is up, negative is down)
            roll_value = 1.0 * self.joystick_msg.axes[RIGHT_STICK_Y]
            # Update the angular roll based on the right stick's Y-axis value
            self.twist_msg.twist.angular.y = roll_value
            
            return True

    
    def JoystickMsg(self, msg):
        
        self.joystick_msg = msg
    
def main(args=None):
    rclpy.init(args=args)

   
    kipps_arm_node = Arm_Control()

    rclpy.spin(kipps_arm_node)

    kipps_arm_node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()