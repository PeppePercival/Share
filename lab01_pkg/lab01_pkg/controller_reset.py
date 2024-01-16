import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ControllerResetNode(Node):
    #resets the controller to make the robot go to the original position

    def __init__(self):
        super().__init__('controller_reset')
        self.publisher = self.create_publisher(Twist, '/cmd_topic', 10)
        self.subscription = self.create_subscription(Bool, '/reset', self.reset_callback, 10)
        timer_period = 1  # 1Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.side_length = 1  # Initial side length
        self.i = 1 #N strats from 1
        
    def reset_callback(self,msg):
        if msg.data:
            self.get_logger().info("Resetting the controller")
            self.i = 1
            self.side_length = 1

    def timer_callback(self):
        msg = Twist()
        speed = 1  # Constant speed of 1 m/s
        print('----------------------------------')
        print(self.i)
        print(self.side_length)
        print('----------------------------------')
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


        #else:
            # Completed one square spiral, increase the side length
            #self.side_length += 1
            #self.i = 0  # Reset counter to restart the movement

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Twist: {msg}')
        self.get_logger().debug('Timer callback executed')

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    controller_reset_node = ControllerResetNode()
    rclpy.spin(controller_reset_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_reset_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
