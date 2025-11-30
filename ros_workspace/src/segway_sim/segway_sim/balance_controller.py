#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

def quaternion_to_euler(qx, qy, qz, qw):
    # returns roll, pitch, yaw
    # source: standard conversion
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        print("Initialising balance controller......")
        self.declare_parameter('Kp', 6.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.6)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.max_speed = self.get_parameter('max_speed').value

        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.target_pitch = 0.0  # desired upright angle
        self.prev_time = None
        self.prev_error = 0.0
        self.integral = 0.0

        # Optionally use gyro for derivative term
        self.prev_pitch = 0.0

    def imu_cb(self, msg: Imu):
        # extract pitch
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        _, pitch, _ = quaternion_to_euler(qx, qy, qz, qw)

        # compute dt
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            dt = 0.01
        else:
            dt = max(1e-6, now - self.prev_time)
        self.prev_time = now

        # error = pitch - desired
        error = pitch - self.target_pitch

        # integrate & derivative
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # PID output (control variable -> linear velocity)
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # limit
        linear_speed = max(-self.max_speed, min(self.max_speed, u))

        # publish as twist (no angular z)
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        # update
        self.prev_error = error
        self.prev_pitch = pitch

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
