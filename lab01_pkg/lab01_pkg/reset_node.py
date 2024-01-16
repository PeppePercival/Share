import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool 

class ResetNode(Node):
        def __init__(self):
                super().__init__('reset')
                self.subscription = self.create_subscription(
                        Pose, '/pose', self.pose_callback, 10)
                self.publisher = self.create_publisher(Bool, '/reset', 10)
        def calculate_distance(self,x,y):
                return ((x ** 2) + (y ** 2)) ** 0.5  # Euclidean distance from origin
        

        def pose_callback(self, msg):
                dist = self.calculate_distance(msg.position.x, msg.position.y)
                if dist > 6:
                        reset_msg = Bool()
                        reset_msg.data = True
                        self.publisher.publish(reset_msg)
                        self.get_logger().info(f'Reset: {reset_msg.data}')
        

def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetNode()
    rclpy.spin(reset_node)
    reset_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


