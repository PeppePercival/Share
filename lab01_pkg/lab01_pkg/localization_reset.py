import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool


class LocalizationResetNode(Node):

    def __init__(self):
        super().__init__('localization_reset')
        self.subscription_pose = self.create_subscription(
            Twist, '/cmd_topic', self.cmd_topic_callback, 10)
        self.publisher_pose = self.create_publisher(Pose, '/pose', 10)
        self.subscription_reset = self.create_subscription(Bool, '/reset', self.reset_callback, 10)
        self.x = 0.0
        self.y = 0.0
    
    def cmd_topic_callback(self, msg):
        pose_msg = Pose()
        self.x += msg.linear.x
        self.y += msg.linear.y  
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        self.publisher_pose.publish(pose_msg)
        self.get_logger().info(f'Published Pose: x={self.x}, y={self.y}')

        self.get_logger().debug('Publishing pose executed')

    def pose_calback(self,msg):
        self.x = msg.position.x
        self.y = msg.position.y
    
    def reset_callback(self,msg):
        if msg.data:
            self.x = 0.0
            self.y = 0.0
            reset_msg = Pose()
            reset_msg.position.x = self.x
            reset_msg.position.y = self.y
            self.publisher_pose.publish(reset_msg)
            self.get_logger().info('Position reset to origin')

def main(args=None):
    rclpy.init(args=args)
    localization_reset_node = LocalizationResetNode()
    rclpy.spin(localization_reset_node)
    localization_reset_node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()

