import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
import matplotlib.pyplot as plt
import cv2
import numpy as np
import io

class LIDARVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            '/lidar_image',
            10
        )

    def lidar_callback(self, msg):
        angles = np.arange(len(msg.ranges)) * msg.angle_increment
        distances = np.array(msg.ranges)
        
        # Generate plot
        plt.figure(figsize=(10, 5))
        plt.plot(angles, distances)
        plt.title('LIDAR Data Plot')
        plt.xlabel('Angle (radians)')
        plt.ylabel('Distance (meters)')
        plt.grid(True)
        
        # Convert plot to image
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        plt.close()
        buf.seek(0)
        
        # Convert image to ROS message
        image_msg = self.convert_plot_to_ros_image(buf)
        self.publisher.publish(image_msg)

    def convert_plot_to_ros_image(self, buf):
        # Load image from buffer
        img_array = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        
        # Create ROS Image message
        msg = Image()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = 'bgr8'
        msg.step = img.shape[1] * img.shape[2]
        msg.data = img.tobytes()
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = LIDARVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
