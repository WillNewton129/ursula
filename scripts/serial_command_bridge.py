#!/usr/bin/env python3
"""
serial_command_bridge.py

Converts /cmd_vel (Twist) to Arduino serial commands.
Publishes /ursula/bridge_state for Foxglove monitoring.
Subscribes to /ursula/estop (Bool) for remote emergency stop.

Serial format sent to Arduino:
    CMD,<left_pwm>,<right_pwm>,<estop>\n
    estop: 1 = normal operation, 0 = emergency stop

SAFETY CAPS:
    max_linear_vel  (default 0.3 m/s) — clips cmd_vel.linear.x before
    max_angular_vel (default 0.5 rad/s) — clips cmd_vel.angular.z before
    kinematic conversion.  This prevents Nav2 or any other source from
    ever commanding more than a safe wheel speed regardless of input.

    With the defaults and turn_multiplier=4.5, wheel_base=0.56:
        max_wheel_vel = 0.3 + (0.5 * 4.5 * 0.28) = 0.3 + 0.63 = 0.93
        max_PWM = 0.93 * 255 ≈ 237  (safely below 255)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray
import serial

MAX_PWM  = 80
SEND_HZ  = 20


class SerialCmdBridge(Node):
    def __init__(self):
        super().__init__('serial_cmd_bridge')

        # ----------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------
        self.declare_parameter('serial_port',     '/dev/ttyACM0')
        self.declare_parameter('baud_rate',        115200)
        self.declare_parameter('wheel_base',       0.56)
        self.declare_parameter('estop_button',     4)
        self.declare_parameter('turn_multiplier',  4.5)
        # Safety velocity caps — applied BEFORE kinematic conversion.
        # These prevent any source (Nav2, teleop) from exceeding safe speeds.
        self.declare_parameter('max_linear_vel',   0.3)   # m/s
        self.declare_parameter('max_angular_vel',  0.5)   # rad/s
        self.declare_parameter('min_angular_vel', 0.05)
        self.declare_parameter('angular_deadband', 0.005)

        serial_port        = self.get_parameter('serial_port').value
        baud_rate          = self.get_parameter('baud_rate').value
        self.wheel_base    = self.get_parameter('wheel_base').value
        self.estop_button  = self.get_parameter('estop_button').value
        self.turn_multiplier   = self.get_parameter('turn_multiplier').value
        self.max_linear_vel    = self.get_parameter('max_linear_vel').value
        self.max_angular_vel   = self.get_parameter('max_angular_vel').value
        self.min_angular_vel = self.get_parameter('min_angular_vel').value
        self.angular_deadband = self.get_parameter('angular_deadband').value

        # ----------------------------------------------------------------
        # Serial connection
        # ----------------------------------------------------------------
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to Arduino on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial = None

        # ----------------------------------------------------------------
        # State
        # ----------------------------------------------------------------
        self.linear_vel   = 0.0
        self.angular_vel  = 0.0
        self.estop_active = 1   # 1 = stopped / safe,  0 = go

        # ----------------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------------
        # /ursula/bridge_state  — Float32MultiArray for Foxglove plot panel
        # data[0] = left_pwm
        # data[1] = right_pwm
        # data[2] = estop_active (1=stopped, 0=running)
        # data[3] = linear_vel command (m/s)
        # data[4] = angular_vel command (rad/s)
        self.bridge_state_pub = self.create_publisher(
            Float32MultiArray, '/ursula/bridge_state', 10
        )

        # ----------------------------------------------------------------
        # Subscriptions
        # ----------------------------------------------------------------
        self.create_subscription(Twist, '/cmd_vel',       self.cmd_vel_callback, 10)
        self.create_subscription(Joy,   '/joy',            self.joy_callback,     10)
        self.create_subscription(Bool,  '/ursula/estop',  self.remote_estop_callback, 10)

        # ----------------------------------------------------------------
        # Send timer
        # ----------------------------------------------------------------
        self.timer = self.create_timer(1.0 / SEND_HZ, self.send_command)

        self.get_logger().info('Serial command bridge started')
        self.get_logger().info(
            f'turn_multiplier={self.turn_multiplier}  '
            f'max_linear={self.max_linear_vel} m/s  '
            f'max_angular={self.max_angular_vel} rad/s  '
            f'min_angular={self.min_angular_vel} rad/s  '
            f'angular_deadband={self.angular_deadband} rad/s'
        )

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------  
    def cmd_vel_callback(self, msg: Twist):
     # Clamp linear velocity
        self.linear_vel = max(
        -self.max_linear_vel,
        min(self.max_linear_vel, msg.linear.x)
        )

    # Clamp angular velocity first
        angular = max(
        -self.max_angular_vel,
        min(self.max_angular_vel, msg.angular.z)
        )

    # Lift small non-zero angular commands above the motor deadband
    # but preserve true zero as zero.
        if abs(angular) > self.angular_deadband and abs(angular) < self.min_angular_vel:
            angular = self.min_angular_vel if angular > 0 else -self.min_angular_vel

        self.angular_vel = angular

    def joy_callback(self, msg: Joy):
        """Physical LB button (button 4) estop — read directly from joystick."""
        if len(msg.buttons) > self.estop_button:
            button_pressed    = msg.buttons[self.estop_button]
            self.estop_active = 0 if button_pressed else 1

    def remote_estop_callback(self, msg: Bool):
        """Remote estop via /ursula/estop Bool topic (e.g. from Foxglove Publish panel).
        Publish True  → activate stop (estop_active = 1)
        Publish False → release stop  (estop_active = 0)
        """
        if msg.data:
            self.estop_active = 1
            self.get_logger().warn('REMOTE ESTOP ACTIVATED via /ursula/estop')
        else:
            self.estop_active = 0
            self.get_logger().info('Remote estop released via /ursula/estop')

    # ----------------------------------------------------------------
    # Kinematics
    # ----------------------------------------------------------------
    def diff_drive_kinematics(self, linear, angular):
        """Convert cmd_vel to left/right wheel velocities.

        For skid-steer, we amplify the angular component because the wheels
        must skid sideways to change heading — they cannot roll freely.
        """
        angular_adjusted = angular * self.turn_multiplier
        left_vel  = linear - (angular_adjusted * self.wheel_base / 2.0)
        right_vel = linear + (angular_adjusted * self.wheel_base / 2.0)
        return left_vel, right_vel

    # ----------------------------------------------------------------
    # Send loop (20 Hz)
    # ----------------------------------------------------------------
    def send_command(self):
        if not self.serial or not self.serial.is_open:
            return

        left_vel, right_vel = self.diff_drive_kinematics(
            self.linear_vel, self.angular_vel
        )

        # Clamp to [-1, 1] then scale to PWM integer
        left_pwm  = int(max(-1.0, min(1.0, left_vel))  * MAX_PWM)
        right_pwm = int(max(-1.0, min(1.0, right_vel)) * MAX_PWM)
        MIN_PWM = 25

        if left_pwm != 0 and abs(left_pwm) < MIN_PWM:
            left_pwm = MIN_PWM if left_pwm > 0 else -MIN_PWM

        if right_pwm != 0 and abs(right_pwm) < MIN_PWM:
            right_pwm = MIN_PWM if right_pwm > 0 else -MIN_PWM

        # Publish monitoring state for Foxglove
        state_msg = Float32MultiArray()
        state_msg.data = [
            float(left_pwm),
            float(right_pwm),
            float(self.estop_active),
            self.linear_vel,
            self.angular_vel,
        ]
        self.bridge_state_pub.publish(state_msg)

        # Format: CMD,left_pwm,right_pwm,estop\n
        command = f'CMD,{left_pwm},{right_pwm},{self.estop_active}\n'
        try:
            self.serial.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ----------------------------------------------------------------
    # Cleanup
    # ----------------------------------------------------------------
    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.write(b'CMD,0,0,1\n')   # Guaranteed stop on shutdown
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