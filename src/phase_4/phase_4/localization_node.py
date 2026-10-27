import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class LocalizationNode(Node):
    """Convert GPS coordinates to local ENU and publish world -> car TF."""

    def __init__(self):
        super().__init__('localization_node')

        self.declare_parameter('datum_latitude', 45.61898065946136)
        self.declare_parameter('datum_longitude', 9.28112698795041)
        self.declare_parameter('datum_altitude', 176.10797824886478)
        self.declare_parameter('fix_topic', '/COM/fix')
        self.declare_parameter('orientation_topic', '/COM/compass')

        self.datum_lat = math.radians(
            self.get_parameter('datum_latitude').value
        )
        self.datum_lon = math.radians(
            self.get_parameter('datum_longitude').value
        )
        self.datum_alt = float(
            self.get_parameter('datum_altitude').value
        )
        self.earth_radius = 6378137.0

        self.orientation = None

        self.tf_broadcaster = TransformBroadcaster(self)

        fix_topic = self.get_parameter('fix_topic').value
        orientation_topic = self.get_parameter('orientation_topic').value

        self.fix_sub = self.create_subscription(
            NavSatFix, fix_topic, self.fix_callback, 10
        )
        self.orientation_sub = self.create_subscription(
            Imu, orientation_topic, self.orientation_callback, 10
        )

        self.get_logger().info(
            f'Listening to {fix_topic} and {orientation_topic}'
        )

    def orientation_callback(self, msg):
        q = msg.orientation

        # Reject unavailable or invalid quaternions.
        if msg.orientation_covariance[0] == -1.0:
            return

        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            return

        self.orientation = (
            q.x / norm,
            q.y / norm,
            q.z / norm,
            q.w / norm,
        )

    def fix_callback(self, msg):
        if msg.status.status < 0 or self.orientation is None:
            return

        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # Local tangent-plane approximation: x=east, y=north.
        east = (
            self.earth_radius
            * (lon - self.datum_lon)
            * math.cos(self.datum_lat)
        )
        north = self.earth_radius * (lat - self.datum_lat)

        if math.isfinite(msg.altitude):
            up = msg.altitude - self.datum_alt
        else:
            up = 0.0

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'world'
        tf.child_frame_id = 'car'

        tf.transform.translation.x = east
        tf.transform.translation.y = north
        tf.transform.translation.z = up

        qx, qy, qz, qw = self.orientation
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()