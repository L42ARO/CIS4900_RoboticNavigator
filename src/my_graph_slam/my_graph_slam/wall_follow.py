
import rclpy
from rclpy.node import Node
from .robot_controller import RobotController
import numpy as np

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.robot_controller = RobotController(self, True, False)
        self.create_timer(0.1, self.follow_wall)  # Run the follow_wall method every 0.1 seconds

        # PID parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.4  # Integral gain
        self.kd = 0.2  # Derivative gain
        self.integral = 0
        self.previous_error = 0

        # Parameters
        self.desired_distance = 0.4  # Desired distance from the wall
        self.stop_threshold = 0.2  # Distance at which the robot should stop to avoid collision

    def follow_wall(self):
        

        if not self.robot_controller.lidar_ready():
            print("LIDAR NOT READY")
            return  # LIDAR data not ready yet

        angles, distances = self.robot_controller.get_lidar_data()
        angle_min = self.robot_controller.lidar_data.angle_min
        angle_increment = self.robot_controller.lidar_data.angle_increment


        # Replace 'inf' with 3 in distances array
        distances = np.nan_to_num(distances, nan=3, posinf=3)

        front_start_angle = np.deg2rad(170)
        front_end_angle = np.deg2rad(190)
        front_start_index = int((front_start_angle - angle_min) / angle_increment)
        front_end_index = int((front_end_angle - angle_min) / angle_increment)
        front_distances = distances[front_start_index:front_end_index]
        if(np.mean(front_distances)<self.stop_threshold):
            self.robot_controller.stop()
            return
        
            

        # Calculate indices for obstacle detection between 120 and 140 degrees
        obstacle_start_angle = np.deg2rad(115)
        obstacle_end_angle = np.deg2rad(145)
        obstacle_start_index = int((obstacle_start_angle - angle_min) / angle_increment)
        obstacle_end_index = int((obstacle_end_angle - angle_min) / angle_increment)

        # Check for obstacles and adjust PID error
        obstacle_distances = distances[obstacle_start_index:obstacle_end_index]
        obstacle_influence = 0
        front_influence = 0
        if np.mean(obstacle_distances) < 0.4:
            obstacle_influence = -0.5  # This value can be adjusted based on desired influence
            print("Obstacle detected, adjusting error for left steering")
        elif(np.mean(front_distances)<0.5):
            front_influence = -0.9

        # Calculate indices for averaging between 80 and 100 degrees
        start_angle = np.deg2rad(80)
        end_angle = np.deg2rad(100)
        start_index = int((start_angle - angle_min) / angle_increment)
        end_index = int((end_angle - angle_min) / angle_increment)

        # Average distance calculation
        average_distance = np.mean(distances[start_index:end_index])

        # Error calculation adjusted for obstacle influence
        error = average_distance - self.desired_distance + obstacle_influence + front_influence

        # PID calculations
        self.integral += error * 0.1  # assuming timer calls this at 10Hz
        derivative = (error - self.previous_error) / 0.1
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

        # Use the output to control the robot
        turn = -output  # Negative sign because a positive error means too far from the wall
        print(f'Turn: {turn}')
        self.robot_controller.move([0.2, turn])  # Move forward and turn based on PID output

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        print("Interrupted by Ctrl+C, stopping robot...")
        wall_follower.robot_controller.stop()
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
