import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjust the frequency as needed
        self.image_path = './rumours.jpg'  # Path to the image

    def timer_callback(self):
        # Read the image using OpenCV
        cv_img = cv2.imread(self.image_path)
        if cv_img is None:
            self.get_logger().error(f"Failed to read image from {self.image_path}")
            return
        
        # Convert the OpenCV image to a ROS2 message
        img_msg = self.cv2_to_imgmsg(cv_img, encoding="bgr8")

        # Publish the image
        self.publisher.publish(img_msg)
        self.get_logger().info('Publishing image')

    def cv2_to_imgmsg(self, cv_img, encoding="bgr8"):
        img_msg = Image()
        img_msg.height = cv_img.shape[0]
        img_msg.width = cv_img.shape[1]
        img_msg.encoding = encoding
        img_msg.is_bigendian = False
        img_msg.step = int(cv_img.strides[0])
        img_msg.data = cv_img.tobytes()
        return img_msg

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
