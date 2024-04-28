from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import numpy as np

class RobotController:
    def __init__(self, node, lidar=True, depth_image=True):
        self.node = node
        self.publisher = node.create_publisher(Twist, 'cmd_vel', 10)
        if(lidar):
            self.lidar_subscriber = node.create_subscription(
                LaserScan, '/scan', self.lidar_callback, 10)
        self.lidar_data = None
        if depth_image:
            self.sub_depth = node.create_subscription(Image,"/camera/depth/image_raw", self.depth_img_callback, 1)
        self.encoding = ['16UC1', '32FC1']
        self.latest_depth_frame = None

    def lidar_callback(self, msg):
        # Callback function to update LIDAR data
        self.lidar_data = msg
        
    def depth_img_callback(self, msg):
        if not isinstance(msg, Image): return
        depthFrame = self.node.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding[1])
        self.latest_depth_frame = depthFrame
    def depth_frame_ready(self):
        return self.latest_depth_frame is not None
    def get_depth_frame(self):
        return self.latest_depth_frame

    def lidar_ready(self):
        # Check if LIDAR data is available
        return self.lidar_data is not None

    def get_lidar_data(self):
        # Get and return the current LIDAR data (angles and distances)
        angles = np.arange(
            self.lidar_data.angle_min, self.lidar_data.angle_max, self.lidar_data.angle_increment)
        distances = np.array(self.lidar_data.ranges)
        return angles, distances

    def move(self, vector):
        # Create and publish a Twist message to move the robot
        twist = Twist()
        twist.linear.x = vector[0]  # linear velocity
        twist.angular.z = vector[1]  # angular velocity
        self.publisher.publish(twist)

    def stop(self):
        # Stop the robot by publishing zero velocities
        stop_twist = Twist()  # Create a Twist message with default values (0)
        self.publisher.publish(stop_twist)
