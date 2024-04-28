from flask import Flask, render_template_string, Response
from flask_socketio import SocketIO
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
import base64
import io

app = Flask(__name__)
socketio = SocketIO(app)
messages = []
latest_image = None

@app.route('/')
def index():
    return render_template_string('''
        <!doctype html>
<html>
<head>
    <title>ROS2 Flask Server</title>
</head>
<body>
    <img id="rosImage1" src="" alt="ROS Image"> <!-- Ensure this ID matches the JavaScript access -->
    <img id="rosImage2" src="" alt="ROS Image"> <!-- Ensure this ID matches the JavaScript access -->
    <img id="rosImage3" src="" alt="ROS Image"> <!-- Ensure this ID matches the JavaScript access -->
    <img id="rosImage4" src="" alt="ROS Image"> <!-- Ensure this ID matches the JavaScript access -->
    <h1>ROS Messages</h1>
    <div id="messages">
        <!-- Messages will be displayed here -->
    </div>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.0/socket.io.js"></script>
    <script type="text/javascript">
        var socket = io();
        socket.on('connect', function() {
            console.log('Connected!');
        });
        socket.on('message', function(msg) {
            var elem = document.createElement('p');
            elem.appendChild(document.createTextNode(msg));
            document.getElementById('messages').appendChild(elem);
        });
        socket.on('update_image', function(image_src) {
            var channel = image_src[0];
            var image_src_parsed = image_src.slice(1);
            var imageElement = document.getElementById('rosImage'+channel);
            if (imageElement) {
                imageElement.src = image_src_parsed;
            } else {
                console.log('Image element not found!');
            }
        });
    </script>
</body>
</html>

        ''', image_src=latest_image)

class ROS2Node(Node):
    def __init__(self):
        super().__init__('flask_ros2_node')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.image_sub1 = self.create_subscription(Image, 'server_image_1', self.image1_callback, 10)
        self.image_sub2 = self.create_subscription(Image, 'server_image_2', self.image2_callback, 10)
        self.image_sub3 = self.create_subscription(Image, 'server_image_3', self.image3_callback, 10)
        self.image_sub4 = self.create_subscription(Image, 'server_image_4', self.image4_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        messages.append(msg.data)
        socketio.emit('message', msg.data)

    def image1_callback(self, img_msg):
        self.image_callback(img_msg, "1")
    def image2_callback(self, img_msg):
        self.image_callback(img_msg, "2")
    def image3_callback(self, img_msg):
        self.image_callback(img_msg, "3")
    def image4_callback(self, img_msg):
        self.image_callback(img_msg, "4")

    def image_callback(self, img_msg, channel):
        print(f"Sending image to {channel}")
        cv2_image = self.convert_ros_image_to_cv2(img_msg)
    
        # Determine the file extension and encoding parameters
        is_png = 'png' in img_msg.encoding
        ext = 'png' if is_png else 'jpg'
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90] if not is_png else []

        # Correct usage of cv2.imencode: it returns a tuple (success, buffer)
        success, buffer = cv2.imencode(f'.{ext}', cv2_image, encode_param)
        if not success:
            raise ValueError("Image encoding failed")

        # Encode the buffer to base64
        encoded_image = base64.b64encode(buffer).decode('utf-8')

        # Prepare the data URI for the image
        mime_type = 'image/png' if is_png else 'image/jpeg'
        latest_image = f"{channel}data:{mime_type};base64,{encoded_image}"
        socketio.emit('update_image', latest_image)



    def convert_ros_image_to_cv2(self, img_msg):
        # Determine the dtype based on the encoding
        if '8' in img_msg.encoding:
            dtype = np.uint8
        elif '16' in img_msg.encoding:
            dtype = np.uint16
        else:
            raise ValueError(f"Unsupported encoding: {img_msg.encoding}")

        # Calculate the number of channels
        if 'rgb' in img_msg.encoding or 'bgr' in img_msg.encoding:
            channels = 3
        elif 'rgba' in img_msg.encoding or 'bgra' in img_msg.encoding:
            channels = 4
        else:
            channels = 1  # Assume mono if not specified

        # Check if the buffer size matches the expected size (height * width * channels * bytes_per_channel)
        expected_size = img_msg.height * img_msg.width * channels * dtype().itemsize
        if len(img_msg.data) != expected_size:
            raise ValueError("The buffer size does not match the expected image size")

        # Create the NumPy array for the image
        frame = np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, channels)

        # Convert color format if needed
        if img_msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif img_msg.encoding == 'rgba8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGRA)

        return frame

def run_app():
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False)

def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = ROS2Node()
    t1 = threading.Thread(target=run_app)
    t2 = threading.Thread(target=ros_spin, args=(node,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()

if __name__ == '__main__':
    main()
