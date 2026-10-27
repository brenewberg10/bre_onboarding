import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int8
from autonoma_msgs.msg import VehicleInputs


class VehicleInputAdapter(Node):
    """Combine controller commands into the PAIRsim VehicleInputs message."""

    def __init__(self):
        super().__init__('vehicle_input_adapter')

        self.declare_parameter('steering_ratio', 15.015)
        self.declare_parameter('max_steering_wheel_angle', 240.0)

        self.steering_ratio = float(
            self.get_parameter('steering_ratio').value
        )
        self.max_steering = float(
            self.get_parameter('max_steering_wheel_angle').value
        )

        # Safe initial values until controller commands arrive.
        self.steering_front_wheel_deg = 0.0
        self.throttle = 0.0
        self.brake_kpa = 0.0
        self.gear = 1

        self.throttle_count = 0
        self.brake_count = 0
        self.steering_count = 0

        self.pub = self.create_publisher(
            VehicleInputs, '/vehicle_inputs', 10
        )

        self.create_subscription(
            Float32, '/joystick/steering_cmd',
            self.steering_callback, 10
        )
        self.create_subscription(
            Float32, '/joystick/accelerator_cmd',
            self.throttle_callback, 10
        )
        self.create_subscription(
            Float32, '/joystick/brake_cmd',
            self.brake_callback, 10
        )
        self.create_subscription(
            Int8, '/joystick/gear_cmd',
            self.gear_callback, 10
        )

        self.get_logger().info(
            'Combining steering, throttle, brake, and gear '
            'commands into /vehicle_inputs'
        )

    @staticmethod
    def finite_or_zero(value):
        return float(value) if math.isfinite(float(value)) else 0.0

    def steering_callback(self, msg):
        self.steering_front_wheel_deg = self.finite_or_zero(msg.data)
        self.publish_inputs()

    def throttle_callback(self, msg):
        self.throttle = max(0.0, min(100.0, self.finite_or_zero(msg.data)))
        self.publish_inputs()

    def brake_callback(self, msg):
        self.brake_kpa = max(0.0, self.finite_or_zero(msg.data))
        self.publish_inputs()

    def gear_callback(self, msg):
        self.gear = max(1, min(6, int(msg.data)))
        self.publish_inputs()

    def publish_inputs(self):
        msg = VehicleInputs()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # KinControl's angle is the front-wheel angle in degrees.
        # PAIRsim expects the steering-wheel angle.
        steering_wheel_deg = (
            self.steering_front_wheel_deg * self.steering_ratio
        )
        steering_wheel_deg = max(
            -self.max_steering,
            min(self.max_steering, steering_wheel_deg)
        )

        self.throttle_count = (self.throttle_count + 1) % 256
        self.brake_count = (self.brake_count + 1) % 256
        self.steering_count = (self.steering_count + 1) % 256

        msg.throttle_cmd = float(self.throttle)
        msg.throttle_cmd_count = self.throttle_count

        msg.brake_cmd = float(self.brake_kpa)
        msg.brake_cmd_count = self.brake_count

        msg.steering_cmd = float(steering_wheel_deg)
        msg.steering_cmd_count = self.steering_count

        msg.gear_cmd = self.gear

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInputAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
