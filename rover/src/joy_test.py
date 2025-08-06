#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyTestNode(Node):
    def __init__(self):
        super().__init__('joy_test_node')
        
        # Track previous states to only show changes
        self.prev_buttons = []
        self.prev_axes = []
        
        # Subscribe to joy topic
        self.joy_subscription = self.create_subscription(
            Joy,
            'manual/joy2',
            self.joy_callback,
            10)
        
        self.get_logger().info("Joy Test Node started. Press buttons and move sticks to see mappings...")
        self.get_logger().info("Listening on topic: manual/joy2")
        
    def joy_callback(self, msg):
        # Initialize previous states if first message
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
        if not self.prev_axes:
            self.prev_axes = [0.0] * len(msg.axes)
        
        # Check for button changes
        for i, button in enumerate(msg.buttons):
            if i < len(self.prev_buttons) and button != self.prev_buttons[i]:
                if button == 1:
                    self.get_logger().info(f"BUTTON {i} PRESSED")
                else:
                    self.get_logger().info(f"BUTTON {i} RELEASED")
        
        # Check for significant axis changes (deadzone of 0.1)
        for i, axis in enumerate(msg.axes):
            if i < len(self.prev_axes):
                if abs(axis - self.prev_axes[i]) > 0.1:
                    self.get_logger().info(f"AXIS {i}: {axis:.3f}")
        
        # Update previous states
        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)
        
        # Also print raw message occasionally for reference
        if hasattr(self, '_counter'):
            self._counter += 1
        else:
            self._counter = 1
            
        if self._counter % 100 == 1:  # Print every 100 messages
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"Total buttons: {len(msg.buttons)}")
            self.get_logger().info(f"Total axes: {len(msg.axes)}")
            self.get_logger().info(f"Current button states: {list(msg.buttons)}")
            self.get_logger().info(f"Current axis values: {[f'{x:.3f}' for x in msg.axes]}")
            self.get_logger().info("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = JoyTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()