import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute


class SquareDrawer(Node):
    def __init__(self, side_len=2.0, fwd_speed=2.0, turn_speed=1.57):
        super().__init__('square_drawer')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self._clear_screen()
        self._teleport_to(5.5445, 5.5445, 0.0)

        self.side_len = side_len
        self.fwd_speed = fwd_speed
        self.turn_speed = turn_speed

        self.draw_square()

        self.get_logger().info('Done.')
        rclpy.shutdown()

    def _clear_screen(self):
        client = self.create_client(Empty, '/clear')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /clear...')
        client.call_async(Empty.Request())
        time.sleep(0.1)

    def _teleport_to(self, x, y, theta):
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/teleport_absolute...')
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        client.call_async(req)
        time.sleep(0.1)

    def _drive_forward(self, distance, speed):
        duration = distance / speed
        msg = Twist()
        msg.linear.x = speed
        t0 = time.time()
        while time.time() - t0 < duration:
            self.pub.publish(msg)
            time.sleep(0.01)
        msg.linear.x = 0.0
        self.pub.publish(msg)
        time.sleep(0.05)

    def _turn_in_place(self, angle_rad, ang_speed):
        duration = abs(angle_rad) / ang_speed
        msg = Twist()
        msg.angular.z = ang_speed if angle_rad >= 0 else -ang_speed
        t0 = time.time()
        while time.time() - t0 < duration:
            self.pub.publish(msg)
            time.sleep(0.01)
        msg.angular.z = 0.0
        self.pub.publish(msg)
        time.sleep(0.05)

    def draw_square(self):
        for _ in range(4):
            self._drive_forward(self.side_len, self.fwd_speed)
            self._turn_in_place(math.pi / 2.0, self.turn_speed)


def main():
    rclpy.init()
    node = SquareDrawer(side_len=2.5, fwd_speed=2.0, turn_speed=1.57)
    rclpy.spin(node)


if __name__ == '__main__':
    main()