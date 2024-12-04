import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile

Joy_Topic = "/SPEAR_Arm/Joy_Topic"
Twist_Topic = "/SPEAR_Arm/delta_twist_cmds"
Joint_Topic = "/SPEAR_Arm/delta_joint_cmds"
EEF_Frame_ID = "EEF"
BASE_FRAME_ID = "base_link"


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

        
        self.joy_sub = self.create_subscription(msg_type = Joy, topic = Joy_Topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.JoystickMsg)
        
        
        timer_period = 0.5

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.JoyMain)
        
        self.joystick_msg = Joy()


    def JoyMain(self):


        self.twist_msg = TwistStamped()
        self.joint_msg = JointJog()
        
        

        if(self.ConvertJoyToCommand()):

            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_msg.header.frame_id = "base_link"
            self.twist_pub.publish(self.twist_msg)


        else:
            
            self.joint_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_msg.header.frame_id = "base_link"
            self.joint_pub.publish(self.joint_msg)


    def ConvertJoyToCommand(self):

        if(self.joystick_msg.buttons[A] or self.joystick_msg.buttons[B] or self.joystick_msg.buttons[X] 
        or self.joystick_msg.buttons[Y] or self.joystick_msg.axes[D_PAD_X] or self.joystick_msg.buttons[LEFT_STICK_CLICK]
        or self.joystick_msg.axes[D_PAD_Y] or self.joystick_msg.buttons[RIGHT_STICK_CLICK] or self.joystick_msg.buttons[CHANGE_VIEW]
        or self.joystick_msg.buttons[MENU]):


            #Shoulder Roll 
            self.joint_msg.joint_names.append("Shoulder Roll")
            self.joint_msg.velocities.append(self.joystick_msg.axes[D_PAD_X])
            
            #Shoulder Pitch 
            self.joint_msg.joint_names.append("Shoulder Pitch")
            self.joint_msg.velocities.append(self.joystick_msg.axes[D_PAD_Y])

            
            self.joint_msg.joint_names.append("Elbow Roll")
            self.joint_msg.velocities.append(self.joystick_msg.buttons[B] - self.joystick_msg.buttons[X])


            self.joint_msg.joint_names.append("Elbow Pitch")
            self.joint_msg.velocities.append(self.joystick_msg.buttons[Y] - self.joystick_msg.buttons[A])

            self.joint_msg.joint_names.append("Wrist Roll")
            self.joint_msg.velocities.append(self.joystick_msg.buttons[RIGHT_STICK_CLICK]- self.joystick_msg.buttons[LEFT_STICK_CLICK])

            self.joint_msg.joint_names.append("Wrist Pitch")
            self.joint_msg.velocities.append(self.joystick_msg.buttons[CHANGE_VIEW]- self.joystick_msg.buttons[MENU])
            return False 

        self.twist_msg.twist.linear.z = -(self.joystick_msg.axes[LEFT_STICK_Y])
        self.twist_msg.twist.linear.y = self.joystick_msg.axes[LEFT_STICK_X]
        
        
        lin_x_right = -0.5 * (self.joystick_msg.axes[RIGHT_TRIGGER]- Axis_Default["RIGHT_TRIGGER"])
        lin_x_left = 0.5 * (self.joystick_msg.axes[LEFT_TRIGGER]- Axis_Default["LEFT_TRIGGER"])
        self.twist_msg.twist.linear.x = lin_x_right + lin_x_left

        self.twist_msg.twist.angular.y = self.joystick_msg.axes[RIGHT_STICK_Y]
        self.twist_msg.twist.angular.x = self.joystick_msg.axes[RIGHT_STICK_X]
        
        roll_postive = 1.0 * self.joystick_msg.buttons[RIGHT_BUMPER]
        roll_negative = (-1.0) * self.joystick_msg.buttons[LEFT_BUMPER]
        self.twist_msg.twist.angular.y = roll_negative + roll_postive

        return True

    
    def JoystickMsg(self, msg):
        
        self.joystick_msg = msg
    



def main(args=None):
    rclpy.init(args=args)

   
    SPEAR_Arm_Node = Arm_Control()

    rclpy.spin(SPEAR_Arm_Node)

    SPEAR_Arm_Node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()