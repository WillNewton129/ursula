#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import serial
import math

class SerialCmdBridge(Node):
    def __init__(self):
        super().__init__('serial_cmd_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.56)  # 560mm width
        self.declare_parameter('estop_button', 4)   # LB button (button 4)
        self.declare_parameter('turn_multiplier', 3.0)  # Amplify turning for skid-steer
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.estop_button = self.get_parameter('estop_button').value
        self.turn_multiplier = self.get_parameter('turn_multiplier').value
        
        # Serial connection
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to Arduino on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial = None
        
        # State
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.estop_active = 1  # Default: estop ON (safe state)
        
        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        # Timer to send commands at fixed rate
        self.timer = self.create_timer(0.05, self.send_command)  # 20Hz
        
        self.get_logger().info('Serial command bridge started')
        self.get_logger().info(f'Turn multiplier: {self.turn_multiplier}')
    
    def cmd_vel_callback(self, msg):
        """Receive cmd_vel from teleop"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def joy_callback(self, msg):
        """Monitor estop button from joystick"""
        # Button 4 (LB): when pressed (1), estop is OFF (send 0)
        # When not pressed (0), estop is ON (send 1)
        if len(msg.buttons) > self.estop_button:
            button_pressed = msg.buttons[self.estop_button]
            self.estop_active = 0 if button_pressed else 1
    
    def diff_drive_kinematics(self, linear, angular):
        """Convert cmd_vel to left/right wheel velocities
        
        For skid-steer robots, we amplify the angular component
        because wheels need to skid rather than roll freely.
        """
        # Apply turn multiplier to angular velocity for skid-steering
        angular_adjusted = angular * self.turn_multiplier
        
        # Standard differential drive equations
        left_vel = linear - (angular_adjusted * self.wheel_base / 2.0)
        right_vel = linear + (angular_adjusted * self.wheel_base / 2.0)
        
        return left_vel, right_vel
    
    def send_command(self):
        """Send command to Arduino"""
        if self.serial is None or not self.serial.is_open:
            return
        
        # Convert to differential drive
        left_vel, right_vel = self.diff_drive_kinematics(
            self.linear_vel, self.angular_vel)
        
        # Format: CMD,left_vel,right_vel,estop\n
        command = f"CMD,{left_vel:.3f},{right_vel:.3f},{self.estop_active}\n"
        
        try:
            self.serial.write(command.encode('utf-8'))
            # self.get_logger().debug(f'Sent: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()