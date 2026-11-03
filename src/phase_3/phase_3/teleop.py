import rclpy
from rclpy.node import Node
from autonoma_msgs.msg import VehicleInputs
from pynput import keyboard


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.publisher = self.create_publisher(
            VehicleInputs, '/vehicle_inputs', 10
        )

        # Conservative starting commands
        self.max_throttle = 20.0
        self.steering_angle = 30.0
        self.brake_pressure = 100.0

        # Current keyboard state
        self.forward = False
        self.reverse_brake = False
        self.left = False
        self.right = False
        self.space = False
        self.exit_requested = False

        # Message command counters
        self.throttle_count = 0
        self.brake_count = 0
        self.steering_count = 0

        # Publish commands at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_command)

        # Listen for keyboard events
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        self.get_logger().info(
            'Keyboard teleoperation started.\n'
            'W: throttle | S: brake | A/D: steer | '
            'Space: brake | Esc: stop and exit'
        )

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.forward = True
            elif key.char == 'a':
                self.left = True
            elif key.char == 'd':
                self.right = True
            elif key.char == 's':
                self.reverse_brake = True

        except AttributeError:
            if key == keyboard.Key.space:
                self.space = True
            elif key == keyboard.Key.esc:
                self.exit_requested = True

    def on_release(self, key):
        try:
            if key.char == 'w':
                self.forward = False
            elif key.char == 'a':
                self.left = False
            elif key.char == 'd':
                self.right = False
            elif key.char == 's':
                self.reverse_brake = False

        except AttributeError:
            if key == keyboard.Key.space:
                self.space = False

    def publish_command(self):
        if self.exit_requested:
            self.publish_stop()
            self.get_logger().info('Stopping teleoperation.')
            rclpy.shutdown()
            return

        msg = VehicleInputs()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Brake takes priority over throttle
        if self.space:
            msg.brake_cmd = self.brake_pressure
            msg.throttle_cmd = 0.0
        elif self.forward:
            msg.throttle_cmd = self.max_throttle
            msg.brake_cmd = 0.0
        else:
            msg.throttle_cmd = 0.0
            msg.brake_cmd = 0.0

        # Steering returns to center when A/D are released
        if self.left:
            msg.steering_cmd = -self.steering_angle
        elif self.right:
            msg.steering_cmd = self.steering_angle
        else:
            msg.steering_cmd = 0.0

        # Start in first gear
        msg.gear_cmd = 1

        self.throttle_count = (self.throttle_count + 1) % 256
        self.brake_count = (self.brake_count + 1) % 256
        self.steering_count = (self.steering_count + 1) % 256

        msg.throttle_cmd_count = self.throttle_count
        msg.brake_cmd_count = self.brake_count
        msg.steering_cmd_count = self.steering_count

        self.publisher.publish(msg)

    def publish_stop(self):
        msg = VehicleInputs()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.throttle_cmd = 0.0
        msg.brake_cmd = self.brake_pressure
        msg.steering_cmd = 0.0
        msg.gear_cmd = 1

        self.throttle_count = (self.throttle_count + 1) % 256
        self.brake_count = (self.brake_count + 1) % 256
        self.steering_count = (self.steering_count + 1) % 256

        msg.throttle_cmd_count = self.throttle_count
        msg.brake_cmd_count = self.brake_count
        msg.steering_cmd_count = self.steering_count

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Attempt to send a final braking command
        if rclpy.ok():
            node.publish_stop()

        node.listener.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
