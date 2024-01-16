import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization')
        self.subscription = self.create_subscription(
            Twist, '/cmd_topic', self.cmd_topic_callback, 10)
        self.publisher = self.create_publisher(Pose, '/pose', 10)
        self.subscription
        self.x = 0.0
        self.y = 0.0
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)


    def cmd_topic_callback(self, msg):
        pose_msg = Pose()
        self.x += msg.linear.x
        self.y += msg.linear.y  
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        self.publisher.publish(pose_msg)
        self.get_logger().debug(f'Published Pose: x={self.x}, y={self.y}')

        #self.get_logger().debug('Publishing pose executed')


def main(args=None):
    rclpy.init(args=args)
    localization_node = LocalizationNode()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
