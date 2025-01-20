import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

Joy_Topic = "/SPEAR_Arm/Joy_Topic"
Twist_Topic = "/SPEAR_Arm/delta_twist_cmds"
Joint_Topic = "/SPEAR_Arm/delta_joint_cmds"
EEF_Frame_ID = "EEF"
BASE_FRAME_ID = "base_link"
EEF_Topic = "/SPEAR_Arm/EEF"

# Initialize joystick commands
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
    "LEFT_TRIGGER": 1.0,
    "RIGHT_TRIGGER": 1.0,
}

class Arm_Control(Node):

    def __init__(self):
        super().__init__('Arm_Control')

        self.frame_to_publish_ = EEF_Frame_ID
        self.in_joint_control_mode = False
        self.in_ik_control_mode = False

        self.twist_pub = self.create_publisher(msg_type=TwistStamped, topic=Twist_Topic, qos_profile=QoSProfile(depth=10))
        self.joint_pub = self.create_publisher(msg_type=JointJog, topic=Joint_Topic, qos_profile=QoSProfile(depth=10))
        self.publisher_ = self.create_publisher(Float32, topic=EEF_Topic, qos_profile=QoSProfile(depth=10))

        self.joy_sub = self.create_subscription(msg_type=Joy, topic=Joy_Topic, qos_profile=rclpy.qos.qos_profile_system_default, callback=self.JoystickMsg)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period_sec=timer_period, callback=self.JoyMain)

        self.joystick_msg = Joy()

    def JoyMain(self):

        self.twist_msg = TwistStamped()
        self.joint_msg = JointJog()

        ik = self.ConvertJoyToCommand()

        if ik and not self.in_joint_control_mode:
            self.publish_twist_command()
        elif not ik and self.in_joint_control_mode:
            self.publish_joint_command()

    def ConvertJoyToCommand(self):

        i = 0
        deadband_threshold = 0.1

        # Check if joystick_msg has the expected number of axes and buttons
        num_axes = len(self.joystick_msg.axes)
        num_buttons = len(self.joystick_msg.buttons)

        # self.get_logger().info(f"Joystick Axes: {num_axes}, Buttons: {num_buttons}")
        # self.get_logger().info(f"Joystick Axes Values: {self.joystick_msg.axes}")
        # self.get_logger().info(f"Joystick Buttons Values: {self.joystick_msg.buttons}")

        if num_axes <= RIGHT_STICK_X or num_buttons <= A:
            self.get_logger().warn("Joystick message does not have expected number of axes or buttons")
            return False

        if abs(self.joystick_msg.axes[RIGHT_STICK_X]) > deadband_threshold and self.joystick_msg.buttons[A]:
            i += 1
            self.joint_msg.joint_names.append("Shoulder Roll")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

        if abs(self.joystick_msg.axes[RIGHT_STICK_X]) > deadband_threshold and self.joystick_msg.buttons[B]:
            i += 1
            self.joint_msg.joint_names.append("Shoulder Pitch")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

        if abs(self.joystick_msg.axes[RIGHT_STICK_X]) > deadband_threshold and self.joystick_msg.buttons[X]:
            i += 1
            self.joint_msg.joint_names.append("Elbow Roll")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

        if abs(self.joystick_msg.axes[RIGHT_STICK_X]) > deadband_threshold and self.joystick_msg.buttons[Y]:
            i += 1
            self.joint_msg.joint_names.append("Elbow Pitch")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

        if abs(self.joystick_msg.axes[RIGHT_STICK_X]) > deadband_threshold and self.joystick_msg.buttons[CHANGE_VIEW]:
            i += 1
            self.joint_msg.joint_names.append("Wrist Roll")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

        if abs(self.joystick_msg.axes[RIGHT_STICK_X]) > deadband_threshold and self.joystick_msg.buttons[MENU]:
            i += 1
            self.joint_msg.joint_names.append("Wrist Pitch")
            self.joint_msg.velocities.append(self.joystick_msg.axes[RIGHT_STICK_X])

        if self.joystick_msg.buttons[RIGHT_BUMPER]:
            i += 1
            eef_open_msg = Float32()
            eef_open_msg.data = self.joystick_msg.axes[RIGHT_BUMPER]
            self.publisher_.publish(eef_open_msg)

        if self.joystick_msg.buttons[RIGHT_TRIGGER]:
            i += 1
            eef_close_msg = Float32()
            eef_close_msg.data = -self.joystick_msg.axes[RIGHT_TRIGGER]
            self.publisher_.publish(eef_close_msg)

        if i > 0:
            self.in_joint_control_mode = True
            self.in_ik_control_mode = False
            return False
        else:
            self.in_joint_control_mode = False
            self.in_ik_control_mode = True
            return True

    def publish_twist_command(self):
        deadband_threshold = 0.1

        self.twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.twist_msg.header.frame_id = "base_link"

        lin_x_left_bumper = (self.joystick_msg.buttons[LEFT_BUMPER])
        lin_x_left_trigger = -0.5 * (self.joystick_msg.axes[LEFT_TRIGGER]+1)
        lin_x = lin_x_left_bumper + lin_x_left_trigger

        if abs(lin_x) > deadband_threshold or abs(self.joystick_msg.axes[LEFT_STICK_Y]) > deadband_threshold or abs(self.joystick_msg.axes[LEFT_STICK_X]) > deadband_threshold:
            self.twist_msg.twist.linear.z = -self.joystick_msg.axes[LEFT_STICK_Y]
            self.twist_msg.twist.linear.y = self.joystick_msg.axes[LEFT_STICK_X]
            self.twist_msg.twist.linear.x = lin_x

            self.twist_msg.twist.angular.y = self.joystick_msg.axes[D_PAD_Y]
            self.twist_msg.twist.angular.x = self.joystick_msg.axes[D_PAD_Y]
            roll_value = 1.0 * self.joystick_msg.axes[RIGHT_STICK_Y]
            self.twist_msg.twist.angular.y = roll_value

            self.get_logger().info(f"Publishing twist command: Linear - x: {self.twist_msg.twist.linear.x}, y: {self.twist_msg.twist.linear.y}, z: {self.twist_msg.twist.linear.z}; Angular - x: {self.twist_msg.twist.angular.x}, y: {self.twist_msg.twist.angular.y}, z: {self.twist_msg.twist.angular.z}")
            self.twist_pub.publish(self.twist_msg)
        else:
            self.get_logger().info("No significant joystick movement detected for IK command.")

    def publish_joint_command(self):
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_msg.header.frame_id = "base_link"
        self.get_logger().info(f"Publishing joint command: {self.joint_msg.joint_names} with velocities: {self.joint_msg.velocities}")
        self.joint_pub.publish(self.joint_msg)

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
