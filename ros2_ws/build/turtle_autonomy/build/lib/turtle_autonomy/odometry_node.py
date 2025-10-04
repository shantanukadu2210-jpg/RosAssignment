import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from turtlesim.msg import Pose
import tf2_ros
import math

def euler_to_quaternion(yaw, pitch, roll):
    # This function converts yaw (theta) to a quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # 1. Broadcasters for TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # 2. Publisher for Odometry
        self.odom_pub = self.create_publisher(Odometry, '/turtle1/odom', 10)

        # 3. Subscriber to the turtle's pose
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publish the static transform from map to odom
        self.publish_static_transform()

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 5.5  # As per assignment
        t.transform.translation.y = 5.5  # As per assignment
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0 # No rotation
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform from map to odom')

    def pose_callback(self, msg: Pose):
        # This function is called every time a new pose is received

        # A. Create the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = msg.x
        odom_msg.pose.pose.position.y = msg.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta (2D) to a quaternion (3D)
        odom_msg.pose.pose.orientation = euler_to_quaternion(msg.theta, 0, 0)

        # Set the velocity
        odom_msg.twist.twist.linear.x = msg.linear_velocity
        odom_msg.twist.twist.angular.z = msg.angular_velocity

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # B. Create the dynamic TF transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
