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
            <h1>ROS Messages</h1>
            <div id="messages">
                <!-- Messages will be displayed here -->
            </div>
            {% if image_src %}
                <img src="{{ image_src }}" alt="ROS Image">
            {% endif %}
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
                    document.querySelector('img').src = image_src;
                });
            </script>
        </body>
        </html>
        ''', image_src=latest_image)

class ROS2Node(Node):
    def __init__(self):
        super().__init__('flask_ros2_node')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, 'image_topic', self.image_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        messages.append(msg.data)
        socketio.emit('message', msg.data)

    def image_callback(self, msg):
        global latest_image
        # Assuming the image message is in a format cv2 can handle
        frame = self.convert_ros_image_to_cv2(msg)
        _, buffer = cv2.imencode('.png', frame)
        encoded_image = base64.b64encode(buffer).decode('utf-8')
        latest_image = f"data:image/png;base64,{encoded_image}"
        socketio.emit('update_image', latest_image)

    def convert_ros_image_to_cv2(self, img_msg):
        height, width = img_msg.height, img_msg.width
        channels = 3 if img_msg.encoding == 'rgb8' else 1
        dtype = np.uint8 if img_msg.encoding == 'rgb8' or img_msg.encoding == 'mono8' else np.float32
        frame = np.ndarray(shape=(height, width, channels), dtype=dtype, buffer=img_msg.data)
        if channels == 1:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
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
