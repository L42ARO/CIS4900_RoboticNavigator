import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.move_turtle)

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Moving the turtle')

def main(args=None):
    rclpy.init(args=args)
    turtle_mover = TurtleMover()
    rclpy.spin(turtle_mover)
    turtle_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
