import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from .robot_controller import RobotController  # Adjust the import based on your project structure
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.pub1 = self.create_publisher(Image, 'server_image_1', 10)
        self.pub2 = self.create_publisher(Image, 'server_image_2', 10)
        self.bridge = CvBridge()
        
        # Initialize the RobotController with the current node
        self.controller = RobotController(self)
        
        # Timer for periodic callback
        self.timer = self.create_timer(0.7, self.timer_callback)

    def timer_callback(self):
        if self.controller.lidar_ready():
            angles, distances = self.controller.get_lidar_data()

            # Create a polar plot
            plt.figure()
            ax = plt.subplot(111, polar=True)
            ax.set_theta_zero_location('S')  # Zero degrees at the bottom
            ax.plot(angles, distances, 'b-')  # Plot data as blue line
            ax.set_title('Lidar Polar Data')
            plt.savefig('/tmp/lidar_polar_plot.png')
            ax.set_theta_direction(-1)  # Angles increase counterclockwise
            plt.close()

            # Load the image from matplotlib
            self.cv_image = cv2.imread('/tmp/lidar_polar_plot.png')
            if self.cv_image is None:
                raise FileNotFoundError("Failed to load the polar plot image.")

            # Convert to ROS image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8")
            self.pub1.publish(ros_image)
            self.get_logger().info('Publishing polar lidar plot')

        if self.controller.depth_frame_ready():
            depth_img = self.controller.get_depth_frame()
            depth_img = self.depth_to_rgb8(depth_img)
            ros_depth_image = self.bridge.cv2_to_imgmsg(depth_img, encoding="rgb8")  # use '32FC1' or '16UC1' based on the output format of your depth sensor
            self.pub2.publish(ros_depth_image)
            self.get_logger().info('Publishing depth frame')

    def depth_to_rgb8(self, depth_image):
        # Assuming depth_image is your depth image array in float32 format (32FC1)
    
        # Normalize the depth image to maximum value of 1.0 for scaling
        depth_normalized = cv2.normalize(depth_image, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    
        # Scale to 255 and convert to uint8
        depth_scaled = (255 * depth_normalized).astype(np.uint8)
    
        # Convert to RGB by replicating the depth across three channels
        depth_rgb = cv2.cvtColor(depth_scaled, cv2.COLOR_GRAY2RGB)
    
        return depth_rgb


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
