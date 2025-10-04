import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class AutonomyNode(Node):
    def __init__(self):
        super().__init__('autonomy_node')

        # --- Parameters ---
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 9.0)
        self.declare_parameter('v_const', 1.5) # Constant linear velocity
        self.declare_parameter('dt', 0.1)     # Time step for prediction

        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.v = self.get_parameter('v_const').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        # --- State ---
        self.current_pose = None

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # --- Controller Timer ---
        self.controller_timer = self.create_timer(0.1, self.run_controller)

        self.get_logger().info(f"Autonomous node started. Navigating to ({self.goal_x}, {self.goal_y})")

    def pose_callback(self, msg: Pose):
        # Store the latest pose of the turtle
        self.current_pose = msg

    def run_controller(self):
        if self.current_pose is None:
            self.get_logger().info("Waiting for first pose message...")
            return

        # Check if we have reached the goal
        dist_to_goal = math.sqrt((self.goal_x - self.current_pose.x)**2 + (self.goal_y - self.current_pose.y)**2)
        if dist_to_goal < 0.2:
            self.get_logger().info("Goal reached!")
            # Stop the robot
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            # You could also shut down the node here if desired
            # rclpy.shutdown() 
            return

        # --- Autonomy Logic from Assignment ---
        best_omega = None
        min_dist_to_goal = float('inf')

        # Sample a range of possible angular velocities
        for omega in [w / 10.0 for w in range(-15, 16)]: # from -1.5 to 1.5 rad/s
            # Predict future position using the unicycle model
            x = self.current_pose.x
            y = self.current_pose.y
            theta = self.current_pose.theta

            x_pred = x + self.v * self.dt * math.cos(theta + (omega / 2.0) * self.dt)
            y_pred = y + self.v * self.dt * math.sin(theta + (omega / 2.0) * self.dt)

            # Calculate distance from predicted position to the goal
            predicted_dist = math.sqrt((self.goal_x - x_pred)**2 + (self.goal_y - y_pred)**2)

            # If this omega gives a better result, store it
            if predicted_dist < min_dist_to_goal:
                min_dist_to_goal = predicted_dist
                best_omega = omega

        # Publish the best command
        cmd_msg = Twist()
        cmd_msg.linear.x = self.v
        cmd_msg.angular.z = float(best_omega)
        self.cmd_vel_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
