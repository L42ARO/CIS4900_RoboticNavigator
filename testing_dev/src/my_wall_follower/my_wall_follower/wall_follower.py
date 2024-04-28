
# ROS libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Common libraries
import math
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.last_error = 0

    def calculate(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.last_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

class WallFollower(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub_laser = self.create_subscription(LaserScan, "/scan", self.laser_callback, 1)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # PID controller for wall following
        self.pid = PID(0.5, 0.01, 0.1, setpoint=1.0)  # Setpoint is the desired distance to the wall
        self.pid.output_limits = (-0.5, 0.5)  # Limits for angular velocity adjustments

        self.timer = self.create_timer(0.1, self.update_cmd)

    def laser_callback(self, msg):
        # Assuming laser scan ranges are an array and the right side is at index 0
        right_distance = msg.ranges[0]  
        self.current_distance = right_distance

    def update_cmd(self):
        if self.current_distance is not None:
            correction = self.pid.calculate(self.current_distance)
            
            twist = Twist()
            twist.linear.x = 0.2  # Constant forward speed
            twist.angular.z = -correction  # Negative correction because a positive should turn the robot to the left
            self.pub_vel.publish(twist)

def main():
    rclpy.init()
    wall_follower = WallFollower("wall_follower_node")
    
    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
