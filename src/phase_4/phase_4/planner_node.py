import csv
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, TransformException


class PlannerNode(Node):
    """Publish a local track path and desired vehicle speed."""

    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter(
            'csv_path',
            '/home/bre/bre_onboarding/map/monza_mincurv.csv'
        )
        self.declare_parameter('datum_latitude', 45.61898065946136)
        self.declare_parameter('datum_longitude', 9.28112698795041)
        self.declare_parameter('path_length_m', 120.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lvms_desired_speed', 25.0)
        self.declare_parameter('max_desired_speed', 62.0)
        self.declare_parameter('speed_profile_scalar', 0.6)

        self.declare_parameter('max_lateral_accel', 2.5)
        self.declare_parameter('max_braking_decel', 4.0)
        self.declare_parameter('speed_preview_distance', 80.0)

        self.datum_lat = math.radians(
            self.get_parameter('datum_latitude').value
        )
        self.datum_lon = math.radians(
            self.get_parameter('datum_longitude').value
        )
        self.earth_radius = 6378137.0
        self.path_length = float(
            self.get_parameter('path_length_m').value
        )
        self.lvms_speed = float(
            self.get_parameter('lvms_desired_speed').value
        )
        self.max_speed = float(
            self.get_parameter('max_desired_speed').value
        )

        self.track = []
        self.track_speeds = []

        self.load_track(self.get_parameter('csv_path').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(
            Path, '/planning/front_path/offset_path', 1
        )
        self.speed_pub = self.create_publisher(
            Float32, '/planning/desired_velocity', 1
        )

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.planning_callback)

        self.get_logger().info(
            f'Loaded {len(self.track)} waypoints; '
            'publishing /planning/front_path/offset_path and '
            '/planning/desired_velocity'
        )

    def latlon_to_enu(self, latitude, longitude):
        lat = math.radians(latitude)
        lon = math.radians(longitude)

        east = (
            self.earth_radius
            * (lon - self.datum_lon)
            * math.cos(self.datum_lat)
        )
        north = self.earth_radius * (lat - self.datum_lat)
        return east, north

    def load_track(self, csv_path):
        default_speed = self.lvms_speed

        with open(csv_path, 'r', newline='') as csv_file:
            reader = csv.reader(csv_file)

            for row in reader:
                if len(row) < 2:
                    continue

                try:
                    latitude = float(row[0].strip())
                    longitude = float(row[1].strip())
                    speed = (
                        float(row[2].strip())
                        if len(row) >= 3 and row[2].strip()
                        else default_speed
                    )
                except (ValueError, IndexError):
                    # Ignore headers or malformed rows.
                    continue

                x, y = self.latlon_to_enu(latitude, longitude)
                self.track.append((x, y))
                self.track_speeds.append(
                    max(0.0, min(speed, self.max_speed))
                )

        if len(self.track) < 2:
            raise RuntimeError(
                f'Could not load enough waypoints from {csv_path}'
            )

    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_curve_speed_limit(self, index):
        """Estimate curvature near a waypoint and calculate a speed limit."""

        n = len(self.track)
        window = 5

        # Select points before, at, and after the waypoint.
        p_before = self.track[(index - window) % n]
        p_center = self.track[index]
        p_after = self.track[(index + window) % n]

        # Calculate the direction of travel on either side.
        heading_before = math.atan2(
            p_center[1] - p_before[1],
            p_center[0] - p_before[0]
        )

        heading_after = math.atan2(
            p_after[1] - p_center[1],
            p_after[0] - p_center[0]
        )

        # Total distance across the two segments.
        distance_before = math.hypot(
            p_center[0] - p_before[0],
            p_center[1] - p_before[1]
        )

        distance_after = math.hypot(
            p_after[0] - p_center[0],
            p_after[1] - p_center[1]
        )

        total_distance = distance_before + distance_after

        if total_distance < 1e-6:
            return self.max_speed

        # Estimate curvature: heading change divided by distance.
        heading_change = abs(
            self.wrap_angle(heading_after - heading_before)
        )

        curvature = heading_change / (total_distance / 2.0)

        # Nearly straight sections have very low curvature.
        if curvature < 1e-4:
            return self.max_speed

        max_lateral_accel = float(
            self.get_parameter('max_lateral_accel').value
        )

        # v = sqrt(a_lat / curvature)
        return math.sqrt(max_lateral_accel / curvature)
    
    def planning_callback(self):
        try:
            # Transform gives the car pose expressed in the world frame.
            tf = self.tf_buffer.lookup_transform(
                'world', 'car', rclpy.time.Time()
            )
        except TransformException:
            return

        car_x = tf.transform.translation.x
        car_y = tf.transform.translation.y
        car_yaw = self.quaternion_to_yaw(tf.transform.rotation)

        # Find the closest track waypoint in world coordinates.
        nearest = min(
            range(len(self.track)),
            key=lambda i: (
                (self.track[i][0] - car_x) ** 2
                + (self.track[i][1] - car_y) ** 2
            )
        )

        # Collect sequential waypoints ahead, wrapping around the track.
        indices = []
        distance = 0.0
        n = len(self.track)

        for step in range(n):
            i = (nearest + step) % n
            j = (i + 1) % n
            indices.append(i)

            x1, y1 = self.track[i]
            x2, y2 = self.track[j]
            distance += math.hypot(x2 - x1, y2 - y1)

            if distance >= self.path_length:
                break

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'base_link'

        cos_yaw = math.cos(car_yaw)
        sin_yaw = math.sin(car_yaw)

        for i in indices:
            j = (i + 1) % n
            world_x, world_y = self.track[i]
            next_x, next_y = self.track[j]

            dx = world_x - car_x
            dy = world_y - car_y

            # Rotate world displacement into the car frame.
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy

            tangent_yaw = math.atan2(
                next_y - world_y,
                next_x - world_x
            )
            local_yaw = self.wrap_angle(tangent_yaw - car_yaw)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = local_x
            pose.pose.position.y = local_y
            pose.pose.position.z = 0.0

            # Planar quaternion for the track tangent in base_link.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(local_yaw / 2.0)
            pose.pose.orientation.w = math.cos(local_yaw / 2.0)

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # Overall speed multiplier.
        speed_scalar = float(
            self.get_parameter('speed_profile_scalar').value
        )

        # Maximum assumed braking deceleration.
        max_braking_decel = float(
            self.get_parameter('max_braking_decel').value
        )

        # Distance ahead over which to look for corners.
        preview_distance = float(
            self.get_parameter('speed_preview_distance').value
        )

        # Start with the speed allowed at the current waypoint.
        desired_speed = min(
            self.track_speeds[nearest] * speed_scalar,
            self.get_curve_speed_limit(nearest)
        )

        # Look ahead for corners and slow down before reaching them.
        distance_ahead = 0.0
        n = len(self.track)

        for step in range(1, n):
            previous_index = (nearest + step - 1) % n
            future_index = (nearest + step) % n

            x1, y1 = self.track[previous_index]
            x2, y2 = self.track[future_index]

            distance_ahead += math.hypot(x2 - x1, y2 - y1)

            if distance_ahead > preview_distance:
                break

            # Speed limit at the upcoming waypoint.
            future_speed = min(
                self.track_speeds[future_index] * speed_scalar,
                self.get_curve_speed_limit(future_index)
            )

            # Maximum current speed that permits braking to the future limit.
            braking_speed = math.sqrt(
                future_speed ** 2
                + 2.0 * max_braking_decel * distance_ahead
            )

            desired_speed = min(desired_speed, braking_speed)

        # Publish the final speed target.
        speed_msg = Float32()
        speed_msg.data = float(desired_speed)
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
