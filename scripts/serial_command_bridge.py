#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import serial

MAX_PWM = 255  # Scale factor from m/s to PWM (adjust if needed)
SEND_HZ = 20

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
        self.estop_active = 1  # 1 = stopped, 0 = go
        
        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / SEND_HZ, self.send_command)
        self.get_logger().info('Serial command bridge started')
        self.get_logger().info(f'Turn multiplier: {self.turn_multiplier}')
    
    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def joy_callback(self, msg):
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
        if not self.serial or not self.serial.is_open:
            return
        
        left_vel, right_vel = self.diff_drive_kinematics(self.linear_vel, self.angular_vel)
        
        # Convert to integer PWM
        left_pwm = int(max(-1.0, min(1.0, left_vel)) * MAX_PWM)
        right_pwm = int(max(-1.0, min(1.0, right_vel)) * MAX_PWM)
        
        # Format: CMD,left_pwm,right_pwm,estop\n
        command = f"CMD,{left_pwm},{right_pwm},{self.estop_active}\n"
        
        try:
            self.serial.write(command.encode('utf-8'))
            self.get_logger().info(f'Sent: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def destroy_node(self):
        if self.serial and self.serial.is_open:
            # Stop motors before closing
            self.serial.write(b'CMD,0,0,1\n')
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
