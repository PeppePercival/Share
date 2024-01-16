import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_topic', 10)
        timer_period = 1  # 1Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.side_length = 1  # Initial side length
        self.i = 1  # Counter to track how much the robot has moved
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    def timer_callback(self):
        msg = Twist()
        speed = 1  # Constant speed of 1 m/s

        if self.i <= self.side_length:
            msg.linear.x = float(speed)  # Move along the X-axis
        elif self.i <= 2 * self.side_length:
            msg.linear.y = float(speed)  # Move along the Y-axis
        elif self.i <= 3 * self.side_length:
            msg.linear.x = -float(speed)  # Move opposite the X-axis
        elif self.i <= 4 * self.side_length:
            msg.linear.y = -float(speed)  # Move opposite the Y-axis
            if self.i == 4 *self.side_length:
                self.side_length += 1
                self.i = 0  # Reset counter to restart the movement

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing Twist: {msg}')
        #self.get_logger().debug('Timer callback executed')

        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
